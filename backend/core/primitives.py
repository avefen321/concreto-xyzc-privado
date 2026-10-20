"""Machine geometry, independent of the display tessellation."""
from dataclasses import dataclass, field, asdict
import math
from .gcode import arc_geometry


@dataclass(frozen=True)
class MotionPrimitive:
    id: str
    source_id: int | None
    layer_id: int | None
    stroke_id: int | None
    type: str
    start_xyz: tuple
    end_xyz: tuple
    start_c: float
    end_c: float
    feed: float | None
    process_state: str
    source_line: int
    sample_id: int
    anchor_line: int | None = None
    motion_type: str = 'LINEAR'
    generated_reason: str = 'ORIGINAL'
    E: float | None = None
    metadata: dict = field(default_factory=dict)

    @property
    def X(self): return self.end_xyz[0]
    @property
    def Y(self): return self.end_xyz[1]
    @property
    def Z(self): return self.end_xyz[2]
    @property
    def C_deg(self): return self.end_c
    @property
    def feed_mm_min(self): return self.feed
    @property
    def length_mm(self): return math.dist(self.start_xyz,self.end_xyz)
    @property
    def duration_s(self): return 60*self.length_mm/self.feed if self.feed and self.type!='TRAVEL' else None
    @property
    def required_c_velocity_deg_s(self): return abs(self.end_c-self.start_c)/self.duration_s if self.duration_s else None

    def pose(self,t):
        return (*tuple(a+(b-a)*t for a,b in zip(self.start_xyz,self.end_xyz)),self.start_c+(self.end_c-self.start_c)*t)

    def to_dict(self):
        return dict(asdict(self),length_mm=self.length_mm,duration_s=self.duration_s,
                    required_c_velocity_deg_s=self.required_c_velocity_deg_s)


@dataclass(frozen=True)
class ArcPrimitive(MotionPrimitive):
    plane: str = 'G17'
    center_xyz: tuple = (0.,0.,0.)
    radius: float = 0.
    sweep_rad: float = 0.
    direction: str = 'CCW'
    start_angle_rad: float = 0.
    arc_source: str = 'SOURCE_G2G3'

    @property
    def end_angle_rad(self): return self.start_angle_rad+self.sweep_rad
    @property
    def length_mm(self): return math.hypot(self.radius*self.sweep_rad,self.end_xyz[2]-self.start_xyz[2])

    def pose(self,t):
        if t==0: return (*self.start_xyz,self.start_c)
        if t==1: return (*self.end_xyz,self.end_c)
        a=self.start_angle_rad+self.sweep_rad*t
        return (self.center_xyz[0]+self.radius*math.cos(a),self.center_xyz[1]+self.radius*math.sin(a),
                self.start_xyz[2]+(self.end_xyz[2]-self.start_xyz[2])*t,
                self.start_c+(self.end_c-self.start_c)*t)

    def tangent_xy(self,t):
        a=self.start_angle_rad+self.sweep_rad*t+math.copysign(math.pi/2,self.sweep_rad)
        return [math.cos(a),math.sin(a)]

    def to_dict(self):
        return dict(super().to_dict(),end_angle_rad=self.end_angle_rad,
                    sweep_deg=math.degrees(self.sweep_rad),
                    tangent_start_xy=self.tangent_xy(0),tangent_end_xy=self.tangent_xy(1),
                    I=self.center_xyz[0]-self.start_xyz[0],J=self.center_xyz[1]-self.start_xyz[1])


def native_enabled(config):
    return config.native_arcs.enabled and config.serialization.motion_mode=='HYBRID_NATIVE_CURVES' and config.sampling.mode!='legacy'


def stroke_display_points(stroke,config):
    result=[list(stroke.points[0].xyz)]
    for a,b in zip(stroke.points,stroke.points[1:]):
        if b.arc_center is None: result.append(list(b.xyz)); continue
        angle=math.atan2(a.xyz[1]-b.arc_center[1],a.xyz[0]-b.arc_center[0])
        sweep=math.radians(b.arc_sweep_deg)
        chord=2*math.acos(max(-1.,1-min(config.sampling.geometry_error_mm,config.corner_smoothing.max_chord_error_mm)/b.arc_radius))
        steps=max(1,math.ceil(abs(sweep)/min(chord,math.radians(config.sampling.max_delta_c_deg))))
        if steps+len(result)>config.sampling.max_samples_per_stroke:
            raise ValueError('max_samples_per_stroke superado por display de arco original')
        for k in range(1,steps+1):
            t=k/steps
            result.append(list(b.xyz) if k==steps else [b.arc_center[0]+b.arc_radius*math.cos(angle+sweep*t),
                          b.arc_center[1]+b.arc_radius*math.sin(angle+sweep*t),a.xyz[2]+(b.xyz[2]-a.xyz[2])*t])
    return result


def arc_parts(point):
    sweep=abs(point.arc_sweep_deg)
    # LinuxCNC accepts one complete center-format circle (I/J, P defaults to 1).
    # Keep that analytic source motion as one machine primitive; tessellation is
    # still used for the viewer.  Multi-turn source arcs remain quartered until
    # the serializer emits their explicit P turn count.
    if point.arc_source=='SOURCE_G2G3' and sweep>360+1e-8:
        return max(1,math.ceil(sweep/90-1e-10))
    return 1


def use_native_arc(point,config):
    source=point.arc_source or 'CORNER_FILLET'
    if source=='SOURCE_G2G3': return config.native_arcs.source_arcs.preserve
    if source=='FREE_CURVE_FIT': return config.native_arcs.fitted_free_curves.enabled
    return config.native_arcs.corner_fillets.enabled


def build_machine_primitives(fields,motions,samples,config):
    """Arc geometry comes from Field, never reconstructed from viewer points.

    Adaptive straight intervals share their validated endpoints with display.
    Only their endpoints are machine commands; arc tessellation is display-only.
    """
    primitives=[]; previous_xyz=(0.,0.,0.); previous_c=0.
    for s in samples:
        if not s.emit: continue
        geometry=None; ta=0.; tb=1.
        if native_enabled(config) and s.stroke_id in fields and s.motion_type=='LINEAR':
            f=fields[s.stroke_id]; p=f.stroke.points[s.field_segment+1]
            if p.arc_center is not None and use_native_arc(p,config):
                parts=arc_parts(p); rank=round(s.t*parts)
                if rank==0 or abs(s.t-rank/parts)>1e-8:
                    s.emit=False; s.display_only=True; continue
                ta,tb=(rank-1)/parts,rank/parts
                start=f.xyz_at(s.field_segment,ta); end=f.xyz_at(s.field_segment,tb)
                center=(*p.arc_center,start[2]); sweep=math.radians(p.arc_sweep_deg)*(tb-ta)
                geometry=(start,end,center,p.arc_radius,sweep,p.arc_source or 'CORNER_FILLET',p.fit_metadata)
                start_c=f.at(s.field_segment,ta)
        elif native_enabled(config) and config.native_arcs.source_arcs.preserve and s.motion_type in ('ARC_CW','ARC_CCW'):
            m=motions[int(s.motion_index)]; cx,cy,r,a0,sweep=arc_geometry(m)
            sweep_deg=abs(math.degrees(sweep))
            parts=max(1,math.ceil(sweep_deg/90-1e-10)) if sweep_deg>360+1e-8 else 1
            rank=round(s.t*parts)
            if rank==0 or abs(s.t-rank/parts)>1e-8:
                s.emit=False; s.display_only=True; continue
            ta=max(0.,s.t-1/parts)
            start=m.start if ta==0 else (cx+r*math.cos(a0+sweep*ta),cy+r*math.sin(a0+sweep*ta),m.start[2]+(m.end[2]-m.start[2])*ta)
            geometry=(start,(s.X,s.Y,s.Z),(cx,cy,start[2]),r,sweep/parts,'SOURCE_G2G3',{})
            start_c=previous_c
        kind='ARC' if geometry else 'TRAVEL' if s.motion_type in ('RAPID','PREP_RAPID') else 'C_ONLY' if s.motion_type=='C_ONLY' else 'Z_ONLY' if s.generated_reason=='LAYER_TRANSITION' else 'LINE'
        kwargs=dict(id=f'P{len(primitives)+1}',source_id=s.source_motion_id,layer_id=s.layer_id,stroke_id=s.stroke_id,
                    type=kind,start_xyz=previous_xyz,end_xyz=(s.X,s.Y,s.Z),start_c=previous_c,end_c=s.C_deg,
                    feed=s.feed_mm_min,process_state='DEPOSIT' if s.deposition_active else 'TRAVEL',
                    source_line=s.source_line,sample_id=s.sample_id,anchor_line=s.anchor_line,
                    motion_type=s.motion_type,generated_reason=s.generated_reason,E=s.E,
                    metadata={'source_lines':s.source_lines,'fallback':s.primitive_type=='SMOOTH_CURVE',
                              'c_reference_shift_deg':s.c_reference_shift_deg})
        if geometry:
            start,end,center,r,sweep,source,metadata=geometry
            kwargs.update(start_xyz=start,end_xyz=end,start_c=start_c,
                          metadata=dict(metadata,source_lines=s.source_lines,fillet_id=s.fillet_id,
                                        c_reference_shift_deg=s.c_reference_shift_deg),
                          center_xyz=center,radius=r,sweep_rad=sweep,direction='CCW' if sweep>0 else 'CW',
                          start_angle_rad=math.atan2(start[1]-center[1],start[0]-center[0]),arc_source=source)
            primitive=ArcPrimitive(**kwargs)
        else:
            if s.curve_certified and s.stroke_id in fields:
                p=fields[s.stroke_id].stroke.points[s.field_segment+1]
                kwargs['metadata']=dict(p.fit_metadata,**kwargs['metadata'])
            primitive=MotionPrimitive(**kwargs)
        s.primitive_id=primitive.id
        primitives.append(primitive)
        previous_xyz,previous_c=primitive.end_xyz,primitive.end_c
    return tuple(primitives)


def analyze_primitives(primitives,config):
    arcs=[p for p in primitives if p.type=='ARC']; warnings=[]
    limit=config.kinematics.max_c_velocity_deg_s
    for p in arcs:
        if limit and p.required_c_velocity_deg_s is not None and p.required_c_velocity_deg_s>limit+1e-7:
            maximum=60*p.length_mm*limit/abs(p.end_c-p.start_c)
            warnings.append(f'Línea {p.source_line}: ARC_C_VELOCITY_LIMIT: {p.required_c_velocity_deg_s:.3f} °/s; F máxima {maximum:.3f} mm/min')
    metrics={'native_arc_count':len(arcs),'fitted_arc_count':sum(p.arc_source=='FREE_CURVE_FIT' for p in arcs),
             'source_arc_count':sum(p.arc_source=='SOURCE_G2G3' for p in arcs),
             'fillet_arc_count':sum(p.arc_source=='CORNER_FILLET' for p in arcs),
             'fallback_G1_count':sum(p.type=='LINE' and p.metadata.get('fallback',False) for p in primitives),
             'min_radius_mm':min((p.radius for p in arcs),default=None),
             'max_required_omega_c_deg_s':max((p.required_c_velocity_deg_s for p in primitives if p.required_c_velocity_deg_s is not None),default=None),
             'max_fit_error_mm':max((p.metadata.get('max_fit_error_mm',0.) for p in arcs),default=0.),
             'max_tangent_error_deg':max((p.metadata.get('max_tangent_error_deg',0.) for p in arcs),default=0.),
             'max_orientation_fit_error_deg':max((p.metadata.get('max_orientation_fit_error_deg',0.) for p in arcs),default=0.)}
    printed=[p for p in primitives if p.process_state=='DEPOSIT' and p.type in ('LINE','ARC')]
    total=sum(p.length_mm for p in printed)
    metrics['native_arc_length_fraction']=sum(p.length_mm for p in printed if p.type=='ARC')/total if total else 0.
    metrics['coverage_by_stroke']=[]
    for sid in sorted({p.stroke_id for p in printed if p.stroke_id is not None}):
        members=[p for p in printed if p.stroke_id==sid]; length=sum(p.length_mm for p in members)
        cumulative=0.; bins=[0.,0.,0.,0.]
        for p in members:
            if p.type=='ARC' and length:
                for k in range(4):
                    bins[k]+=max(0.,min(cumulative+p.length_mm,length*(k+1)/4)-max(cumulative,length*k/4))
            cumulative+=p.length_mm
        metrics['coverage_by_stroke'].append({'stroke_id':sid,'length_mm':length,
            'arc_count':sum(p.type=='ARC' for p in members),'line_count':sum(p.type=='LINE' for p in members),
            'arc_length_fraction':sum(bins)/length if length else 0.,
            'arc_fraction_by_quarter':[v/(length/4) if length else 0. for v in bins]})
    for key in ('RMS_fit_error_mm','RMS_orientation_fit_error_deg'):
        values=[p.metadata[key] for p in arcs if key in p.metadata]
        metrics[key]=math.sqrt(sum(v*v for v in values)/len(values)) if values else 0.
    # Demand jumps are reported; acceleration requires controller blending time.
    metrics['arc_demand_jumps']=[{'arc_id':p.id,'entry_delta_omega_deg_s':
                                abs((p.required_c_velocity_deg_s or 0)-(primitives[i-1].required_c_velocity_deg_s or 0)) if i else None,
                                'estimated_c_acceleration_deg_s2':None}
                               for i,p in enumerate(primitives) if p.type=='ARC']
    return metrics,warnings
