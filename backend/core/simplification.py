"""Dual-tolerance reduction of existing samples with exact C(s) extrema checks."""
import bisect
import math


def distance_to_chord(p,a,b):
    v=[b[j]-a[j] for j in range(3)]
    vv=sum(x*x for x in v)
    t=max(0.,min(1.,sum((p[j]-a[j])*v[j] for j in range(3))/vv)) if vv else 0.
    return math.dist(p,[a[j]+t*v[j] for j in range(3)])


def simplify(samples, fields, config):
    cfg=config.simplification
    tolerance=min(cfg.geometry_tolerance_mm,config.geometry.max_xy_deviation_mm) if config.geometry.mode=='PRESERVE_PATH' else cfg.geometry_tolerance_mm
    stats={'enabled':cfg.enabled,'input_points':len(samples),'output_points':len(samples),
           'compression_ratio':1.,'max_geometry_error_mm':0.,'max_orientation_error_deg':0.,
           'geometry_tolerance_mm':tolerance,'orientation_tolerance_deg':cfg.orientation_tolerance_deg}
    if not cfg.enabled or len(samples)<3:
        return samples,stats
    keep=set(range(len(samples)))
    groups=[]; current=[]
    def protected(s):
        return (s.curve_certified or s.stroke_id not in fields or s.generated_by_fillet or s.fillet_id is not None or
                s.is_corner or s.motion_type not in ('START','LINEAR') or s.E is not None or
                s.field_segment+1 in fields[s.stroke_id].by_vertex)
    for i,sample in enumerate(samples):
        if current:
            previous=samples[current[-1]]
            if protected(previous) or protected(sample) or previous.stroke_id!=sample.stroke_id or previous.feed_mm_min!=sample.feed_mm_min:
                groups.append(current); current=[]
        current.append(i)
    if current: groups.append(current)
    for group in groups:
        if len(group)<3: continue
        field=fields[samples[group[0]].stroke_id]
        stack=[(0,len(group)-1)]
        accepted=[]
        while stack:
            lo,hi=stack.pop()
            if hi-lo<2: continue
            a,b=samples[group[lo]],samples[group[hi]]
            ds=b.s_stroke-a.s_stroke
            if ds<=config.geometry.zero_length_tolerance_mm: continue
            xyz0=(a.X,a.Y,a.Z); xyz1=(b.X,b.Y,b.Z)
            maxgeom=0.; worst=(lo+hi)//2
            for k in range(lo+1,hi):
                p=samples[group[k]]
                error=distance_to_chord((p.X,p.Y,p.Z),xyz0,xyz1)
                if error>maxgeom: maxgeom=error; worst=k
            maxangle=0.
            first=max(0,bisect.bisect_right(field.s,a.s_stroke+1e-10)-1)
            last=min(len(field.seg_length)-1,bisect.bisect_left(field.s,b.s_stroke)-1)
            for segment in range(first,last+1):
                h=field.seg_length[segment]
                left=max(a.s_stroke,field.s[segment]); right=min(b.s_stroke,field.s[segment+1])
                ca=a.C_deg+(b.C_deg-a.C_deg)*(left-a.s_stroke)/ds
                cb=a.C_deg+(b.C_deg-a.C_deg)*(right-a.s_stroke)/ds
                error=field.interpolation_error(segment,(left-field.s[segment])/h,(right-field.s[segment])/h,ca,cb)
                if error>maxangle:
                    maxangle=error
                    if error/cfg.orientation_tolerance_deg>maxgeom/tolerance:
                        middle=(left+right)/2
                        worst=min(range(lo+1,hi),key=lambda k:abs(samples[group[k]].s_stroke-middle))
            passes=(maxgeom<=tolerance and maxangle<=min(cfg.orientation_tolerance_deg,config.sampling.max_orientation_interp_error_deg)
                    and abs(b.C_deg-a.C_deg)<=config.sampling.max_delta_c_deg
                    and math.dist(xyz0,xyz1)<=config.sampling.max_linear_step_mm)
            if passes:
                for k in range(lo+1,hi): keep.discard(group[k])
                b.generated_reason='SIMPLIFICATION_ENDPOINT'
                b.source_lines=sorted(set(samples[group[k]].source_line for k in range(lo+1,hi+1)))
                b.geometry_error_mm=maxgeom
                b.orientation_error_deg=maxangle
                accepted.append((maxgeom,maxangle))
            else:
                if maxgeom<=tolerance and maxangle<=cfg.orientation_tolerance_deg: worst=(lo+hi)//2
                stack.extend(((worst,hi),(lo,worst)))
        stats['max_geometry_error_mm']=max([stats['max_geometry_error_mm']]+[x[0] for x in accepted])
        stats['max_orientation_error_deg']=max([stats['max_orientation_error_deg']]+[x[1] for x in accepted])
    output=[s for i,s in enumerate(samples) if i in keep]
    stats.update(output_points=len(output),compression_ratio=len(samples)/max(1,len(output)))
    return output,stats
