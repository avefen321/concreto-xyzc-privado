"""Immutable geometry/C reference captured before changing representation.

The domain u is accumulated SOURCE CHORD length, not true curve arc length.
Opaque regions retain the current motor's analytic primitives exactly.
"""
from dataclasses import asdict, dataclass
import json
from functools import cached_property
from .states import BACKEND_VERSION, digest
from .parametric import ParametricReference, BezierPiece


@dataclass(frozen=True)
class ReferenceSnapshot:
    source: str
    config_json: str
    spans_json: str
    baseline_primitives_json: str
    semantic_json: str
    orientation_json: str = '{}'
    backend_version: str = BACKEND_VERSION
    machine_program_json: str = '{}'

    @cached_property
    def hash(self):
        return digest(asdict(self))

    @property
    def config(self):
        from .config import Config
        return Config.from_dict(json.loads(self.config_json))

    @property
    def initial_xyz(self):
        return json.loads(self.machine_program_json).get('initial_xyz')

    def to_dict(self):
        return dict(asdict(self), reference_hash=self.hash, schema_version=1,
                    domain='SOURCE_CHORD_PARAMETER', physical_execution_verified=False)


def span_key(stroke,start,end):
    return digest([stroke.id,start,end,[(p.xyz,p.feed,p.line,p.locked,p.arc_center)
                                       for p in stroke.points]])


def thaw_span(data,stroke=None):
    reference=ParametricReference.__new__(ParametricReference)
    reference.stroke=stroke; reference.start=data['start']; reference.end=data['end']
    reference.pieces=tuple(BezierPiece(**dict(p,controls=tuple(tuple(v) for v in p['controls']))) for p in data['pieces'])
    reference.u=tuple(data['u']); reference.closed=data['closed']
    reference.source_error_bound=data['source_error_bound']
    return reference


def semantic_signature(gcode):
    from .gcode import parse
    # Motion endpoints and feeds are checked against the plan separately.
    return [tuple((k,v) for k,v in b.words if k not in 'XYZCFIJR' and
                  not (k=='G' and v in (0,1,2,3))) for b in parse(gcode)
            if any(k not in 'XYZCFIJR' and not (k=='G' and v in (0,1,2,3)) for k,v in b.words)]


class RepresentationSession:
    def __init__(self,snapshot=None,parameters=None,cancel=None):
        self.snapshot=snapshot
        self.parameters=parameters
        self.cancel=cancel or (lambda:False)
        self.records={} if snapshot is None else json.loads(snapshot.spans_json)
        self.orientations={} if snapshot is None else json.loads(snapshot.orientation_json)
        self.stats={'regions':0,'edges_attempted':0,'edges_accepted':0,'fallback_regions':0}
        self.identity={'representation_version':1,'reference_hash':snapshot.hash if snapshot else None,
                       'parameters':parameters,'capture':snapshot is None}

    def verify(self,source,config):
        if self.snapshot and (source!=self.snapshot.source or config.to_dict()!=self.snapshot.config.to_dict()
                              or self.snapshot.backend_version!=BACKEND_VERSION):
            raise ValueError('La entrada, configuración fija o versión no coincide con ReferenceSnapshot')

    def reference(self,reference,stroke,start,end):
        key=span_key(stroke,start,end)
        if self.snapshot is None:
            self.records[key]={'pieces':[asdict(p) for p in reference.pieces],
                               'u':reference.u,'closed':reference.closed,
                               'source_error_bound':reference.source_error_bound,
                               'start':start,'end':end,'stroke_id':stroke.id,'layer_id':stroke.layer_id,
                               'feed':stroke.points[end].feed}
        elif key in self.records:
            data=self.records[key]
            reference=thaw_span(data,stroke)
        return reference,key

    def fit(self,fitter,a,b,key):
        if self.cancel():
            raise InterruptedError('Búsqueda cancelada')
        if self.parameters is None or key not in self.records:
            return fitter.fit(a,b)
        from .segmentation_optimizer import optimize_interval
        return optimize_interval(fitter,a,b,self.parameters,self.stats,self.cancel)

    def accept(self,key,part):
        for p in part[1:]:
            if p.fit_metadata.get('reference_model')=='PARAMETRIC_HERMITE':
                p.fit_metadata['reference_id']=key

    def align_field(self,stroke_id,field):
        key=str(stroke_id)
        if self.snapshot is None:
            self.orientations[key]={'start_c':field.point_c[0],'end_c':field.point_c[-1]}
            return
        expected=self.orientations[key]
        delta=expected['start_c']-field.point_c[0]
        period=field.config.orientation.symmetry_period_deg
        if abs(delta-round(delta/period)*period)>1e-6:
            raise ValueError('La candidata cambió la orientación del inicio de un trazo')
        # Freeze branch choice, including exact 180-degree ties. Field's
        # polynomial coefficients are derived lazily from point_c.
        field.point_c=[c+delta for c in field.point_c]
        if abs(field.point_c[-1]-expected['end_c'])>1e-6:
            raise ValueError('La candidata cambió el giro continuo total de un trazo')

    def end_c(self,stroke_id,value):
        return self.orientations[str(stroke_id)]['end_c'] if self.snapshot is not None else value


def freeze_reference(source,config,cancel=None):
    from .conversion import convert
    # Keep the user's exact configuration. Without parametric spans the
    # optimizer conserves the analytic machine plan instead of silently
    # replacing POLYLINE_C/PRESERVE_PATH/G1_ONLY by a different curve.
    session=RepresentationSession(cancel=cancel)
    result=convert(source,config,representation=session)
    if result.report['status']=='FAIL' or not result.gcode:
        raise ValueError('La línea base no puede optimizarse: '+'; '.join(result.report['errors']))
    primitives=[p.to_dict() for p in result.final_motion_plan.primitives]
    used={p['metadata'].get('reference_id') for p in primitives}
    spans={k:v for k,v in session.records.items() if k in used}
    for key,data in spans.items():
        first=next(p for p in primitives if p['metadata'].get('reference_id')==key)
        offset=config.orientation_offset_deg+first['metadata'].get('c_reference_shift_deg',0.)
        heading=thaw_span(data).heading(first['metadata']['reference_u_start'])
        period=config.orientation.symmetry_period_deg
        branch=round((first['start_c']-heading-offset)/period)*period
        data['machine_c_offset']=offset+branch
    dump=lambda value:json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
    snapshot=ReferenceSnapshot(source,dump(config.to_dict()),dump(spans),dump(primitives),
                               dump(semantic_signature(result.gcode)),dump(session.orientations),
                               machine_program_json=dump({'gcode':result.gcode,'mapping':result.report['serialization_map'],
                                   'initial_xyz':next(([0.,0.,p['end_xyz'][2]] for p in primitives
                                                      if p['generated_reason']=='INITIAL_XY_APPROACH'),None)}))
    return snapshot,result
