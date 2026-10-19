"""One tolerance contract for analytic arcs and global piecewise curve spans.

Geometry acceptance checks every source vertex (the distance to a segment is
convex on each input line). C acceptance checks the polynomial extrema on every
overlapped field piece. No dense intermediate sample list is constructed.
"""
from dataclasses import dataclass, asdict
import bisect
import math
from .simplification import distance_to_chord


@dataclass
class SmoothCurveSpan:
    start_id: int
    end_id: int
    layer_id: int | None
    stroke_id: int
    s_start: float
    s_end: float
    source_lines: list[int]
    kind: str = 'SMOOTH_CURVE'


class CurveSampler:
    def __init__(self,field,config):
        self.field=field; self.config=config
        self.rows={i:[] for i in range(len(field.seg_length))}
        self.spans=[]
        self.geometry_tolerance=min(config.sampling.geometry_error_mm,config.simplification.geometry_tolerance_mm)
        if config.geometry.mode=='PRESERVE_PATH':
            self.geometry_tolerance=min(self.geometry_tolerance,config.geometry.max_xy_deviation_mm)
        self.angle_tolerance=min(config.sampling.max_orientation_interp_error_deg,config.simplification.orientation_tolerance_deg)

    def location(self,s):
        f=self.field
        nearest=bisect.bisect_left(f.s,s)
        for k in (nearest-1,nearest):
            if 0<k<len(f.s) and abs(f.s[k]-s)<1e-9:
                return k-1,1.
        i=min(len(f.seg_length)-1,max(0,bisect.bisect_left(f.s,s)-1))
        return i,max(0.,min(1.,(s-f.s[i])/f.seg_length[i]))

    def emit(self,sa,sb,geom,angle,kind):
        i,t=self.location(sb); first,_=self.location(sa+1e-10)
        lines=sorted({line for p in self.field.stroke.points[first+1:i+2] for line in (p.source_lines or [p.line])})
        self.rows[i].append({'t':t,'start_s':sa,'geometry_error_mm':geom,
                             'orientation_error_deg':angle,'primitive_type':kind,'source_lines':lines})

    def sample(self):
        f=self.field; n=len(f.seg_length); i=0
        while i<n:
            p=f.stroke.points[i+1]
            if p.arc_center is not None:
                ArcSampler(self).sample(i); i+=1; continue
            end=i+1
            while end<n:
                vertex=f.stroke.points[end]; following=f.stroke.points[end+1]
                if (not self.config.simplification.enabled or vertex.locked or following.locked or
                        vertex.fillet_id is not None or (vertex.heading_override_deg is not None and
                            'reference_orientation_pieces' not in vertex.fit_metadata) or
                        following.arc_center is not None or end in f.by_vertex or end+1 in f.by_vertex or
                        f.stroke.points[end].feed!=following.feed or
                        abs(vertex.xyz[2]-following.xyz[2])>self.config.layers.z_change_tolerance_mm):
                    break
                end+=1
            self.spans.append(asdict(SmoothCurveSpan(i,end,f.stroke.layer_id,f.stroke.id,f.s[i],f.s[end],
                                                    [p.line for p in f.stroke.points[i:end+1]])))
            SmoothCurveSampler(self).sample(f.s[i],f.s[end])
            i=end
        if sum(map(len,self.rows.values()))>self.config.sampling.max_samples_per_stroke:
            raise ValueError('max_samples_per_stroke superado')
        return self.rows,self.spans


class ArcSampler:
    def __init__(self,owner): self.owner=owner

    def sample(self,i):
        o=self.owner; f=o.field; cfg=o.config; arc=f.stroke.points[i+1]
        sweep=abs(math.radians(arc.arc_sweep_deg)); radius=arc.arc_radius
        tolerance=min(cfg.corner_smoothing.max_chord_error_mm,cfg.sampling.geometry_error_mm)
        angle_chord=2*math.acos(max(-1.,min(1.,1-tolerance/radius)))
        delta=math.radians(min(cfg.sampling.max_delta_c_deg,cfg.corner_smoothing.max_delta_c_deg))
        length3=math.hypot(radius*sweep,arc.xyz[2]-f.stroke.points[i].xyz[2])
        angle_length=cfg.sampling.max_linear_step_mm/length3*sweep
        steps=max(1,math.ceil(sweep/min(angle_chord,delta,angle_length)))
        from .primitives import native_enabled, arc_parts
        if native_enabled(cfg):
            parts=arc_parts(arc)
            steps=math.ceil(steps/parts)*parts
        if steps>cfg.sampling.max_samples_per_stroke: raise ValueError('max_samples_per_stroke superado por arco')
        for j in range(1,steps+1):
            sa=f.s[i]+f.seg_length[i]*(j-1)/steps; sb=f.s[i]+f.seg_length[i]*j/steps
            o.emit(sa,sb,radius*(1-math.cos(sweep/steps/2)),0.,'ARC')


class SmoothCurveSampler:
    def __init__(self,owner): self.owner=owner

    def sample(self,sa,sb):
        o=self.owner; f=o.field; cfg=o.config
        stack=[(sa,sb,0)]; accepted=[]
        while stack:
            left,right,depth=stack.pop()
            i,ta=o.location(left); j,tb=o.location(right)
            if ta==1. and i+1<len(f.seg_length): i,ta=i+1,0.
            a,b=f.xyz_at(i,ta),f.xyz_at(j,tb)
            ca,cb=f.at(i,ta),f.at(j,tb); ds=right-left
            geom=angle=0.; worst=(left+right)/2
            first=min(len(f.seg_length)-1,bisect.bisect_right(f.s,left+1e-10)-1)
            for k in range(first,j+1):
                lo=max(left,f.s[k]); hi=min(right,f.s[k+1]); h=f.seg_length[k]
                if hi<=lo: continue
                t0,t1=(lo-f.s[k])/h,(hi-f.s[k])/h
                e=distance_to_chord(f.xyz_at(k,t1),a,b)
                if e>geom:
                    geom=e; worst=hi if hi<right-1e-10 else (left+right)/2
                c0=ca+(cb-ca)*(lo-left)/ds; c1=ca+(cb-ca)*(hi-left)/ds
                ec=f.interpolation_error(k,t0,t1,c0,c1)
                if ec>angle:
                    angle=ec
                    if ec/o.angle_tolerance>geom/o.geometry_tolerance: worst=(lo+hi)/2
            # Only constant-C planar straight spans bypass the curve length cap.
            straight=(geom<=cfg.precision.geometry_epsilon and angle<=cfg.precision.angle_epsilon_deg and
                      abs(cb-ca)<=cfg.precision.angle_epsilon_deg and abs(a[2]-b[2])<=cfg.precision.geometry_epsilon)
            passed=(geom<=o.geometry_tolerance+1e-10 and angle<=o.angle_tolerance+1e-10 and
                    abs(cb-ca)<=cfg.sampling.max_delta_c_deg+1e-9 and
                    (straight or math.dist(a,b)<=cfg.sampling.max_linear_step_mm+1e-9))
            if passed: accepted.append((left,right,geom,angle,'STRAIGHT' if straight else 'SMOOTH_CURVE'))
            else:
                if depth>=40 or ds<=1e-10: raise ValueError('No se puede cumplir el contrato de tolerancias de curva')
                if geom<=o.geometry_tolerance and angle<=o.angle_tolerance: worst=(left+right)/2
                if not left+1e-9<worst<right-1e-9: worst=(left+right)/2
                stack.extend(((worst,right,depth+1),(left,worst,depth+1)))
            if len(stack)+len(accepted)>cfg.sampling.max_samples_per_stroke:
                raise ValueError('max_samples_per_stroke superado por curva')
        for args in sorted(accepted): o.emit(*args)
