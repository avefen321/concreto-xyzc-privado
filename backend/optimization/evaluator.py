"""Independent post-export evaluation against one immutable reference.

Changed smooth spans are re-certified using their frozen Bezier controls.
All other primitives must be unchanged from the baseline, including continuous
C, feed, layers and process. No physical motion or material quality is inferred.
"""
from dataclasses import dataclass
import json
import math
import time
from ..core.reference_snapshot import RepresentationSession, semantic_signature
from ..core.parametric import ParametricReference, BezierPiece
from ..core.curve_reconstruction import ReconstructionFitter
from ..core.arc_fitting import FittedArc
from ..core.conversion import convert
from ..core.gcode import parse, interpret, arc_geometry
from ..core.orientation import nearest_equivalent_angle
from ..core.simplification import distance_to_chord
from ..core.states import digest
from .baseline import exported_metrics


def load_span(data):
    ref=ParametricReference.__new__(ParametricReference)
    ref.pieces=tuple(BezierPiece(**dict(p,controls=tuple(tuple(q) for q in p['controls']))) for p in data['pieces'])
    ref.u=tuple(data['u']); ref.source_error_bound=data['source_error_bound']; ref.closed=data['closed']
    return ref


def opaque_signature(p):
    def stable(value):
        if isinstance(value,float): return round(value,9)
        if isinstance(value,(list,tuple)): return [stable(v) for v in value]
        return value
    return {k:stable(p.get(k)) for k in ('type','start_xyz','end_xyz','start_c','end_c','feed','process_state',
                                       'layer_id','stroke_id','generated_reason','arc_source','center_xyz','sweep_rad','E')}


def line_parameter(ref,a,b,point):
    """Invert monotone chord projection for a sampled sub-block of a G1."""
    start,end=ref.xyz(a),ref.xyz(b)
    d=(end[0]-start[0],end[1]-start[1]); length2=sum(v*v for v in d)
    project=lambda q:sum((q[k]-start[k])*d[k] for k in (0,1))/length2
    target=project(point)
    if target<=1e-10: return a
    if target>=1-1e-10: return b
    left,right=a,b
    for _ in range(55):
        middle=(left+right)/2
        if project(ref.xyz(middle))<target: left=middle
        else: right=middle
    return (left+right)/2


def equivalent(left,right):
    if isinstance(left,(int,float)) and isinstance(right,(int,float)):
        return math.isfinite(left) and math.isfinite(right) and abs(left-right)<=1e-7
    if isinstance(left,dict) and isinstance(right,dict):
        return left.keys()==right.keys() and all(equivalent(left[k],right[k]) for k in left)
    if isinstance(left,(list,tuple)) and isinstance(right,(list,tuple)):
        return len(left)==len(right) and all(equivalent(a,b) for a,b in zip(left,right))
    return left==right


def validate_candidate(snapshot,result):
    config=snapshot.config
    errors=[]; coverage=[]; xy_max=c_max=0.
    spans=json.loads(snapshot.spans_json)
    baseline=json.loads(snapshot.baseline_primitives_json)
    primitives=[p.to_dict() for p in result.final_motion_plan.primitives]
    if result.report['status']=='FAIL' or not result.gcode:
        return {'status':'FAIL','coverage_complete':False,'errors':result.report['errors'] or ['Salida vacía'],
                'xy_error_mm':None,'C_error_deg':None,'regions':[]}
    try:
        motions,_=interpret(parse(result.gcode),initial_xyz=snapshot.initial_xyz)
        if len(motions)!=len(primitives): errors.append('Cantidad de movimientos exportados distinta del plan')
        if result.report['config']!=config.to_dict(): errors.append('Configuración fija modificada')
        if json.loads(json.dumps(semantic_signature(result.gcode)))!=json.loads(snapshot.semantic_json):
            errors.append('Secuencia de comandos no geométricos modificada')
        expected=[opaque_signature(p) for p in baseline if not p['metadata'].get('reference_id')]
        actual=[opaque_signature(p) for p in primitives if not p['metadata'].get('reference_id')]
        if not spans:
            from .plan_compaction import validate_plan_compaction
            opaque_valid=validate_plan_compaction(snapshot,result)
        else:
            opaque_valid=equivalent(actual,expected)
        if not opaque_valid: errors.append('Regiones protegidas o C continuo modificados')
        intervals={key:[] for key in spans}
        for p,m in zip(primitives,motions):
            key=p['metadata'].get('reference_id')
            if not key:
                # Existing round-trip validators check every opaque primitive.
                # Identity to the baseline is an exact conservation proof,
                # not a claim about the original CAD or printed concrete.
                coverage.append({'primitive':p['id'],'status':'PASS' if opaque_valid else 'NOT_EVALUATED',
                                 'kind':'EXACT_ANALYTIC_JOIN' if p['metadata'].get('compacted_primitive_ids') else 'UNCHANGED_ANALYTIC_PRIMITIVE'})
                continue
            if key not in spans: raise ValueError('Referencia de región desconocida')
            data=spans[key]; ref=load_span(data); fitter=ReconstructionFitter(ref,config)
            fitter.angle=config.arc_fit.orientation_tolerance_deg
            lo,hi=(p['metadata'][k] for k in ('reference_u_start','reference_u_end'))
            if p['type']!='ARC':
                full_lo,full_hi=lo,hi
                lo=line_parameter(ref,full_lo,full_hi,p['start_xyz'])
                hi=line_parameter(ref,full_lo,full_hi,p['end_xyz'])
            if not ref.u[0]-1e-8<=lo<hi<=ref.u[-1]+1e-8: raise ValueError('Intervalo de referencia inválido')
            intervals[key].append((lo,hi))
            if p['layer_id']!=data['layer_id'] or p['stroke_id']!=data['stroke_id']:
                raise ValueError('Candidata atraviesa capas o trazos')
            if p['feed']!=data['feed'] or p['process_state']!='DEPOSIT' or p.get('E') is not None:
                raise ValueError('Avance o estado de proceso modificado')
            xyz_start,xyz_end=ref.xyz(lo),ref.xyz(hi)
            endpoint_error=max(math.dist(m.start,xyz_start),math.dist(m.end,xyz_end))
            if p['type']=='ARC' and endpoint_error>math.sqrt(3)*10**(-config.serialization.xyz_decimals)+1e-8:
                raise ValueError('Extremos exportados no coinciden con referencia fija')
            offset=data['machine_c_offset']
            if p['type']=='ARC':
                if not m.kind.startswith('ARC'): raise ValueError('Tipo de arco perdido')
                cx,cy,r,angle,sweep=arc_geometry(m)
                arc=FittedArc(m.start,m.end,(cx,cy),r,sweep,lo,hi)
                fitter.phase_tolerance=1e-7+2*math.asin(min(1.,endpoint_error/max(r,1e-12)))
                valid,reason=fitter.validate_arc(arc)
                if not valid: raise ValueError('Arco exportado no certificado: '+str(reason))
                head=nearest_equivalent_angle(arc.heading,ref.heading(lo),360.)
                start_target=head+offset
                end_target=start_target+math.degrees(sweep)
                c_error=arc.metrics['max_orientation_fit_error_deg']+max(abs(m.c_start-start_target),abs(m.c_end-end_target))
                xy=arc.metrics['max_radial_error_mm']
            else:
                if m.kind!='LINEAR': raise ValueError('Tipo G1 perdido')
                xy=max(distance_to_chord(q,m.start,m.end) for _,controls,_,_ in ref.portions(lo,hi) for q in controls)
                bound=fitter.line_orientation_bound(lo,hi,m.start,m.end,m.c_start-offset,m.c_end-offset)
                if bound is None: raise ValueError('Orientación de G1 exportado no certificada')
                c_error=bound
            xy_max=max(xy_max,xy); c_max=max(c_max,c_error)
            if xy>min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm)+1e-8 or c_error>config.arc_fit.orientation_tolerance_deg+1e-8:
                raise ValueError(f'Cota de error exportado supera el contrato: {p["id"]}, XY={xy}, C={c_error}')
            coverage.append({'primitive':p['id'],'reference_id':key,'u':[lo,hi],'status':'PASS',
                             'kind':'CONTINUOUS_BOUND','xy_bound_mm':xy,'C_bound_deg':c_error})
        for key,parts in intervals.items():
            cursor=spans[key]['u'][0]
            for lo,hi in parts:
                if abs(lo-cursor)>1e-7: raise ValueError('Hueco o solape en cobertura de referencia')
                cursor=hi
            if abs(cursor-spans[key]['u'][-1])>1e-7: raise ValueError('Región de referencia incompleta')
        # Do not trust a copied PASS flag alone: validate the rounded geometry
        # and per-axis endpoints again against the actual FinalMotionPlan.
        from ..core.validators import NativeArcValidator, SerializationValidator
        native=NativeArcValidator.check(result.gcode,result.final_motion_plan.primitives,config)
        serial=SerializationValidator.check(result.gcode,result.report['serialization_map'],result.samples,config)
        for check in (native,serial):
            if check['status']=='FAIL': errors.extend(check['errors'])
        xy_max=max(xy_max,serial['max_xyz_rounding_error_mm']+1e-7)
        c_max=max(c_max,serial['max_c_rounding_error_deg']+1e-7)
        # Endpoint rounding alone does not bound the interior of an opaque
        # native arc. Compare its center, radius and continuous angle law too.
        for p,m in zip(primitives,motions):
            if p['type']!='ARC' or p['metadata'].get('reference_id'): continue
            cx,cy,r,a,sweep=arc_geometry(m)
            delta=(a-p['start_angle_rad']+math.pi)%(2*math.pi)-math.pi
            angular=max(abs(delta),abs(delta+sweep-p['sweep_rad']))
            bound=math.dist((cx,cy),p['center_xyz'][:2])+abs(r-p['radius'])+p['radius']*min(2.,angular)
            xy_max=max(xy_max,bound+1e-7)
        if not primitives: errors.append('Sin movimientos: objetivo no aplicable')
    except (ValueError,KeyError,TypeError,IndexError) as error:
        errors.append(str(error))
    if errors:
        coverage.append({'status':'NOT_EVALUATED','kind':'REMAINING_REGIONS','reason':'; '.join(errors)})
    return {'status':'FAIL' if errors else 'PASS','coverage_complete':not errors,
            'errors':errors,'xy_error_mm':xy_max if not errors else None,'C_error_deg':c_max if not errors else None,
            'xy_error_kind':'CONTINUOUS_BOUND_PLUS_UNCHANGED_PRIMITIVES' if not errors else 'NOT_EVALUATED',
            'regions':coverage,'physical_validation':'NOT_EVALUATED'}


@dataclass
class Evaluation:
    result: object
    metrics: dict


class Evaluator:
    def __init__(self,snapshot,baseline,cancel=None):
        self.snapshot=snapshot; self.baseline=baseline; self.cancel=cancel
        self.cache={}; self.cache_hits=0

    def evaluate(self,parameters=None):
        key=digest([self.snapshot.hash,parameters])
        if key in self.cache:
            self.cache_hits+=1
            return self.cache[key]
        started=time.perf_counter()
        session=RepresentationSession(self.snapshot,parameters,self.cancel)
        try:
            if parameters is None:
                result=self.baseline
            elif not json.loads(self.snapshot.spans_json):
                from .plan_compaction import compact_result
                result,session.stats=compact_result(self.snapshot,self.baseline,parameters,self.cancel)
            else:
                result=convert(self.snapshot.source,self.snapshot.config,representation=session)
            validation=validate_candidate(self.snapshot,result)
            counts=exported_metrics(result.gcode,self.snapshot.config.sampling.preferred_min_block_length_mm,
                                    initial_xyz=self.snapshot.initial_xyz)
            metrics={'reference_hash':self.snapshot.hash,'parameters':parameters,'status':validation['status'],
                     'feasible':validation['status']=='PASS','coverage_complete':validation['coverage_complete'],
                     'N_blocks':counts['motion_blocks'],'M_microblocks':counts['linear_deposition_microblocks'],
                     'arc_microblocks':counts['arc_deposition_microblocks'],'bytes':counts['file_size_bytes'],
                     'xy_error_mm':validation['xy_error_mm'],'C_error_deg':validation['C_error_deg'],
                     'validation':validation,'constructor':session.stats,'t_convert_s':time.perf_counter()-started,
                     't_sim_s':None,'t_dry_run_real_s':None,'t_print_real_s':None}
        except ValueError as error:
            result=None
            metrics={'reference_hash':self.snapshot.hash,'parameters':parameters,'status':'FAIL','feasible':False,
                     'coverage_complete':False,'N_blocks':None,'M_microblocks':None,'arc_microblocks':None,
                     'xy_error_mm':None,'C_error_deg':None,'bytes':None,
                     'validation':{'errors':[str(error)]},'constructor':session.stats,
                     't_convert_s':time.perf_counter()-started,'t_sim_s':None,'t_dry_run_real_s':None,'t_print_real_s':None}
        value=Evaluation(result,metrics)
        self.cache[key]=value
        while len(self.cache)>4:
            self.cache.pop(next(iter(self.cache)))
        return value
