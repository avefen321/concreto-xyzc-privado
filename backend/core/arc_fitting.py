"""Geometric arc/biarc fitting before the final orientation and machine plan.

The reference is the continuous input polyline and its existing C field. For a
source line, a circular candidate's interpolation remainder is bounded by
R * delta_angle**2 / 8. C error is checked at polynomial extrema, not just at
vertices. Neither display sampling density nor serialized text enters this fit.
"""
from dataclasses import dataclass, replace
import bisect
import math
from .orientation import Field, nearest_equivalent_angle, wrap180
from .path import Stroke
from .numerics import compose_affine


@dataclass
class FittedArc:
    start: tuple
    end: tuple
    center: tuple
    radius: float
    sweep: float
    sa: float = 0.
    sb: float = 0.
    metrics: dict | None = None

    @property
    def heading(self):
        return math.degrees(math.atan2(self.start[1]-self.center[1], self.start[0]-self.center[0])) + math.copysign(90,self.sweep)

    def xyz(self,t):
        a=math.atan2(self.start[1]-self.center[1],self.start[0]-self.center[0])+self.sweep*t
        return (self.center[0]+self.radius*math.cos(a), self.center[1]+self.radius*math.sin(a),
                self.start[2]+(self.end[2]-self.start[2])*t)


def circle_through(a,b,c):
    """Translated coordinates avoid cancellation for large machine offsets."""
    ux,uy=b[0]-a[0],b[1]-a[1]; vx,vy=c[0]-a[0],c[1]-a[1]
    det=2*(ux*vy-uy*vx)
    scale=max(math.hypot(ux,uy)*math.hypot(vx,vy),1e-30)
    if abs(det)<1e-10*scale: return None
    u2=ux*ux+uy*uy; v2=vx*vx+vy*vy
    center=(a[0]+(vy*u2-uy*v2)/det,a[1]+(ux*v2-vx*u2)/det)
    return center, math.dist(center,a[:2])


def circular_candidate(points):
    a,c=points[0],points[-1]; b=points[len(points)//2]
    circle=circle_through(a,b,c)
    if circle is None: return None
    center,r=circle; angle=math.atan2(a[1]-center[1],a[0]-center[0]); sweep=0.
    for p in points[1:]:
        following=math.atan2(p[1]-center[1],p[0]-center[0])
        sweep+=math.radians(wrap180(math.degrees(following-angle)))
        angle=following
    if abs(sweep)<1e-12: return None
    return FittedArc(a,c,center,r,sweep)


def tangent_arc(a,b,tangent):
    vx,vy=b[0]-a[0],b[1]-a[1]; nx,ny=-tangent[1],tangent[0]
    denominator=2*(vx*nx+vy*ny)
    if abs(denominator)<1e-10*max(math.hypot(vx,vy),1.): return None
    signed_r=(vx*vx+vy*vy)/denominator
    center=(a[0]+nx*signed_r,a[1]+ny*signed_r)
    alpha=math.atan2(a[1]-center[1],a[0]-center[0])
    beta=math.atan2(b[1]-center[1],b[0]-center[0])
    sweep=(beta-alpha)%(2*math.pi) if signed_r>0 else -((alpha-beta)%(2*math.pi))
    if abs(sweep)<1e-12: return None
    return FittedArc(a,b,center,abs(signed_r),sweep)


def biarc_hermite(a,b,heading_a,heading_b,ratio=1.):
    """Positive-distance Hermite biarc, with d2/d1 as its free parameter.

    The translated join and rationalized quadratic root avoid cancellation.
    ratio=1 retains the original symmetric construction. Only the forward,
    finite branch is useful for machining; other branches are rejected.
    """
    if not math.isfinite(ratio) or ratio<=0: return None
    t0=(math.cos(math.radians(heading_a)),math.sin(math.radians(heading_a)))
    t1=(math.cos(math.radians(heading_b)),math.sin(math.radians(heading_b)))
    v=(b[0]-a[0],b[1]-a[1]); length2=sum(x*x for x in v)
    if length2<1e-18: return None
    dot=sum(v[k]*(t0[k]+ratio*t1[k]) for k in range(2))
    coefficient=2*ratio*max(0.,1-sum(t0[k]*t1[k] for k in range(2)))
    if abs(coefficient)<1e-12:
        if dot<=1e-12: return None
        d=length2/(2*dot)
    else:
        root=math.sqrt(dot*dot+coefficient*length2)
        d=length2/(root+dot) if dot>=0 else (root-dot)/coefficient
    q=(a[0]+(v[0]+ratio*d*(t0[0]-t1[0]))/(1+ratio),
       a[1]+(v[1]+ratio*d*(t0[1]-t1[1]))/(1+ratio),a[2])
    first=tangent_arc(a,q,t0); reverse=tangent_arc(b,q,(-t1[0],-t1[1]))
    if first is None or reverse is None: return None
    second=FittedArc(q,b,reverse.center,reverse.radius,-reverse.sweep)
    if abs(wrap180(first.heading+math.degrees(first.sweep)-second.heading))>1e-7: return None
    return [first,second]


class ArcFitter:
    def __init__(self,reference,config):
        self.f=reference; self.cfg=config; self.logs=[]
        self._candidate_cache={}
        self._line_cost_cache={}
        self.geometry_tolerance=min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm,
                                    config.simplification.geometry_tolerance_mm)
        if config.geometry.mode=='PRESERVE_PATH': self.geometry_tolerance=min(self.geometry_tolerance,config.geometry.max_xy_deviation_mm)
        self.angle_tolerance=min(config.arc_fit.orientation_tolerance_deg,config.simplification.orientation_tolerance_deg,
                                 config.sampling.max_orientation_interp_error_deg)
        self.tangent_tolerance=min(config.arc_fit.tangent_tolerance_deg,self.angle_tolerance)

    def location(self,s):
        i=min(len(self.f.seg_length)-1,max(0,bisect.bisect_right(self.f.s,s)-1))
        return i,max(0.,min(1.,(s-self.f.s[i])/self.f.seg_length[i]))

    def reference_c(self,s):
        i,t=self.location(s); return self.f.at(i,t)

    def reference_xyz(self,s):
        i,t=self.location(s); return self.f.xyz_at(i,t)

    def join_reference_s(self,arcs,start,end):
        """Match the shared tangent to C(s) on an inflection-free span.

        Nearest-XY projection can give a different orientation on coarse data.
        The C root supplies a candidate correspondence, still subject to the
        continuous XYZ and C checks; it does not alter the source field.
        """
        ca=self.reference_c(start); cb=self.reference_c(end)
        target=ca+math.degrees(arcs[0].sweep)
        if abs(cb-ca)<1e-10: return None
        direction=math.copysign(1,cb-ca)
        if not 0 < direction*(target-ca) < direction*(cb-ca): return None
        lo,hi=start,end
        for _ in range(40):
            middle=(lo+hi)/2
            if direction*(self.reference_c(middle)-target)<0: lo=middle
            else: hi=middle
        return (lo+hi)/2

    def continuous_geometry_bound(self,arc,k,lo,hi):
        """Certify line-versus-arc error, refining the bound, not the output.

        On each cell ||E(u)|| <= max(||E(a)||,||E(b)||)+R*dtheta**2/8.
        Bisection tightens that interpolation remainder for coarse source
        chords. A failed endpoint or an unresolved cell remains rejected.
        """
        f=self.f; span=arc.sb-arc.sa
        def error(s):
            return math.dist(f.xyz_at(k,(s-f.s[k])/f.seg_length[k]),arc.xyz((s-arc.sa)/span))
        cells=[(lo,hi,error(lo),error(hi),0)]; maximum=0.
        while cells:
            a,b,ea,eb,depth=cells.pop()
            bound=max(ea,eb)+arc.radius*(arc.sweep*(b-a)/span)**2/8
            if bound<=self.geometry_tolerance or max(ea,eb)>self.geometry_tolerance or depth>=12:
                maximum=max(maximum,bound)
            else:
                middle=(a+b)/2; em=error(middle)
                cells.extend(((a,middle,ea,em,depth+1),(middle,b,em,eb,depth+1)))
        return maximum

    def project_s(self,p,start,end):
        best=(float('inf'),self.f.s[start])
        for k in range(start,end):
            a,b=self.f.stroke.points[k:k+2]; v=[b.xyz[j]-a.xyz[j] for j in range(2)]
            t=max(0.,min(1.,sum((p[j]-a.xyz[j])*v[j] for j in range(2))/sum(x*x for x in v)))
            distance=math.hypot(*(p[j]-a.xyz[j]-t*v[j] for j in range(2)))
            best=min(best,(distance,self.f.s[k]+t*self.f.seg_length[k]))
        return best[1]

    def validate(self,arc):
        cfg=self.cfg.arc_fit; f=self.f
        arc.metrics=None
        if not all(math.isfinite(x) for x in (*arc.center,arc.radius,arc.sweep)) or arc.radius<cfg.min_radius_mm:
            return False,'RADIUS_LIMIT',None
        if cfg.max_radius_mm and arc.radius>cfg.max_radius_mm: return False,'RADIUS_LIMIT',None
        if abs(math.degrees(arc.sweep))>cfg.max_sweep_deg+1e-8: return False,'SWEEP_LIMIT',None
        if arc.sb-arc.sa<=1e-9: return False,'FIT_ERROR',None
        offset=self.cfg.orientation_offset_deg
        ca=nearest_equivalent_angle(arc.heading+offset,self.reference_c(arc.sa),360.)
        cb=ca+math.degrees(arc.sweep)
        tangent=max(abs(ca-self.reference_c(arc.sa)),abs(cb-self.reference_c(arc.sb)))
        if tangent>self.tangent_tolerance+1e-8: return False,'TANGENT_ERROR',None
        geom=radial=angle=0.; distances=[]; angles=[]; worst=(arc.sa+arc.sb)/2; score=0.
        first,_=self.location(arc.sa)
        previous_phi=None; direction=math.copysign(1,arc.sweep)
        for k in range(first,len(f.seg_length)):
            if f.s[k]>=arc.sb-1e-12: break
            lo=max(arc.sa,f.s[k]); hi=min(arc.sb,f.s[k+1])
            if hi<=lo: continue
            ta=(lo-f.s[k])/f.seg_length[k]; tb=(hi-f.s[k])/f.seg_length[k]
            pa,pb=f.xyz_at(k,ta),f.xyz_at(k,tb)
            ua=(lo-arc.sa)/(arc.sb-arc.sa); ub=(hi-arc.sa)/(arc.sb-arc.sa)
            ea=math.dist(pa,arc.xyz(ua)); eb=math.dist(pb,arc.xyz(ub))
            # Certified bound for every point of the original line, including its interior.
            bound=self.continuous_geometry_bound(arc,k,lo,hi)
            geom=max(geom,bound); distances.extend((ea,eb))
            cra=ca+(cb-ca)*ua; crb=ca+(cb-ca)*ub
            error_c=f.interpolation_error(k,ta,tb,cra,crb)
            angle=max(angle,error_c); angles.append(error_c)
            vx,vy=pb[0]-pa[0],pb[1]-pa[1]; vv=vx*vx+vy*vy
            proj=max(0.,min(1.,((arc.center[0]-pa[0])*vx+(arc.center[1]-pa[1])*vy)/vv)) if vv else 0.
            rmin=math.hypot(pa[0]+proj*vx-arc.center[0],pa[1]+proj*vy-arc.center[1])
            radial=max(radial,abs(rmin-arc.radius),abs(math.dist(pa[:2],arc.center)-arc.radius),abs(math.dist(pb[:2],arc.center)-arc.radius))
            phi=math.atan2(pb[1]-arc.center[1],pb[0]-arc.center[0])
            if previous_phi is not None and direction*wrap180(math.degrees(phi-previous_phi)) < -1e-7:
                return False,'POINT_ORDER',None
            previous_phi=phi
            ratio=max(bound/self.geometry_tolerance,error_c/self.angle_tolerance)
            if ratio>score: score=ratio; worst=(lo+hi)/2
        metrics={'max_fit_error_mm':geom,'RMS_fit_error_mm':math.sqrt(sum(e*e for e in distances)/max(1,len(distances))),
                 'max_radial_error_mm':radial,'max_tangent_error_deg':tangent,
                 'max_orientation_fit_error_deg':angle,
                 'RMS_orientation_fit_error_deg':math.sqrt(sum(e*e for e in angles)/max(1,len(angles)))}
        arc.metrics=metrics
        if geom>self.geometry_tolerance or radial>cfg.radial_tolerance_mm: return False,'FIT_ERROR',worst
        if angle>self.angle_tolerance: return False,'TANGENT_ERROR',worst
        return True,None,None

    def candidates(self,start,end):
        return self.span_candidates(self.f.s[start],self.f.s[end])

    def span_candidates(self,sa,sb):
        key=(sa,sb)
        if key not in self._candidate_cache:
            self._candidate_cache[key]=self._span_candidates(sa,sb)
        return self._candidate_cache[key]

    def _span_candidates(self,sa,sb):
        f=self.f
        first=bisect.bisect_right(f.s,sa); last=bisect.bisect_left(f.s,sb)
        points=[self.reference_xyz(sa)]+[f.stroke.points[i].xyz for i in range(first,last)]+[self.reference_xyz(sb)]
        arc=circular_candidate(points)
        reason='FIT_ERROR'; worst=None
        candidates=[]
        if arc:
            arc.sa,arc.sb=sa,sb
            good,reason,worst=self.validate(arc)
            # A single-circle candidate is preferred only when its boundary tangents
            # already agree; otherwise Hermite biarcs avoid accumulating join drift.
            if good and arc.metrics['max_tangent_error_deg']<1e-3: candidates=[arc]
        if not candidates and self.cfg.arc_fit.biarc_enabled:
            best_score=float('inf')
            for ratio in (1.,.5,2.,.25,4.,.125,8.,.75,1.5,.375,3.):
                biarc=biarc_hermite(points[0],points[-1],
                                   self.reference_c(sa)-self.cfg.orientation_offset_deg,
                                   self.reference_c(sb)-self.cfg.orientation_offset_deg,ratio)
                if not biarc or biarc[0].sweep*biarc[1].sweep<=0: continue
                # Try the old geometric correspondence first for dense data.
                start=self.location(sa)[0]; end=self.location(sb-1e-9)[0]+1
                projection=max(sa,min(sb,self.project_s(biarc[0].end,start,end)))
                middles=[projection,self.join_reference_s(biarc,sa,sb)]
                for middle in middles:
                    if middle is None or not sa+1e-9<middle<sb-1e-9: continue
                    biarc[0].sa,biarc[0].sb=sa,middle
                    biarc[1].sa,biarc[1].sb=middle,sb
                    checks=[self.validate(a) for a in biarc]
                    if all(c[0] for c in checks):
                        for a in biarc: a.metrics['biarc_ratio']=ratio
                        return biarc,None,None
                    score=max((max(a.metrics['max_fit_error_mm']/self.geometry_tolerance,
                                   a.metrics['max_orientation_fit_error_deg']/self.angle_tolerance)
                               if a.metrics else float('inf')) for a in biarc)
                    if score<best_score:
                        best_score=score
                        bad=next(c for c in checks if not c[0])
                        reason,worst=bad[1:]
        return candidates,reason,worst

    def point_at(self,s):
        f=self.f; i,t=self.location(s)
        if t<1e-10: template=f.stroke.points[i]
        elif t>1-1e-10: template=f.stroke.points[i+1]
        else: template=replace(f.stroke.points[i+1],xyz=self.reference_xyz(s),locked=False,
                               arc_center=None,arc_radius=None,arc_sweep_deg=None,arc_source=None)
        return replace(template,fit_metadata=dict(template.fit_metadata,reference_s=s),
                       heading_override_deg=self.reference_c(s)-self.cfg.orientation_offset_deg)

    def reference_points(self,sa,sb):
        first=bisect.bisect_right(self.f.s,sa+1e-9); last=bisect.bisect_left(self.f.s,sb-1e-9)
        return [self.point_at(sa)]+[self.point_at(self.f.s[i]) for i in range(first,last)]+[self.point_at(sb)]

    def accepted_points(self,candidates,span_id):
        f=self.f; sa=candidates[0].sa; sb=candidates[-1].sb
        result=[replace(self.point_at(sa),heading_override_deg=candidates[0].heading)]
        first=self.location(sa)[0]; last=self.location(sb-1e-9)[0]+1
        lines=sorted({line for p in f.stroke.points[first+1:last+1] for line in (p.source_lines or [p.line])})
        for candidate in candidates:
            template=self.point_at(candidate.sb)
            metadata=dict(candidate.metrics,source_span=span_id,source_point_count=last-first+1,
                          reference_s_start=candidate.sa,reference_s_end=candidate.sb,reference_s=candidate.sb,
                          candidate_c_start=nearest_equivalent_angle(candidate.heading+self.cfg.orientation_offset_deg,
                                                                    self.reference_c(candidate.sa),360.))
            result.append(replace(template,xyz=candidate.end,arc_center=candidate.center,
                                  arc_radius=candidate.radius,arc_sweep_deg=math.degrees(candidate.sweep),
                                  heading_override_deg=candidate.heading+math.degrees(candidate.sweep),
                                  arc_source='FREE_CURVE_FIT',fit_metadata=metadata,source_lines=lines,locked=True))
        self.logs.append({'span_id':span_id,'source_point_count':last-first+1,'classification':'SMOOTH_CURVE',
                          'split_reason':None,'output':'ARC' if len(candidates)==1 else 'BIARC',
                          'primitive_count':len(candidates),'max_error':max(c.metrics['max_fit_error_mm'] for c in candidates)})
        return result

    def original_block_cost(self,sa,sb):
        from .sampling import CurveSampler,SmoothCurveSampler
        key=(sa,sb)
        if key not in self._line_cost_cache:
            owner=CurveSampler(self.f,self.cfg)
            SmoothCurveSampler(owner).sample(sa,sb)
            self._line_cost_cache[key]=sum(map(len,owner.rows.values()))
        return self._line_cost_cache[key]

    def refine_span(self,sa,sb,span_id,depth=0):
        """Refine in continuous reference arclength, without moving vertices.

        Thirds let a subspan straddle a coarse vertex, unlike vertex-only
        bisection. Failed refinements collapse back to the original points.
        C on every remaining line is restored from the original polynomial.
        """
        log_start=len(self.logs)
        if depth==0 and self.original_block_cost(sa,sb)<=4:
            return self.reference_points(sa,sb)
        candidates,reason,_=self.span_candidates(sa,sb)
        if candidates: return self.accepted_points(candidates,span_id)
        minimum=max(self.cfg.geometry.zero_length_tolerance_mm*10,
                    self.cfg.sampling.preferred_min_block_length_mm/2)
        i,_=self.location(sa); j,_=self.location(sb-1e-9)
        if i==j:
            heading=self.f.seg_unwrapped[i]+self.cfg.orientation_offset_deg
            delta=[wrap180(self.reference_c(s)-heading) for s in (sa,sb)]
            # A forward curve whose allowed tangents stay strictly on one
            # side cannot return to two points on the same straight chord.
            if min(delta)>self.angle_tolerance or max(delta)<-self.angle_tolerance:
                return self.reference_points(sa,sb)
        if depth>=self.cfg.arc_fit.max_refinement_depth or sb-sa<minimum:
            return self.reference_points(sa,sb)
        cuts=[sa,sa+(sb-sa)/3,sa+2*(sb-sa)/3,sb]; result=[]
        for k,(a,b) in enumerate(zip(cuts,cuts[1:])):
            part=self.refine_span(a,b,span_id+f'V{k}',depth+1)
            if result:
                result[-1]=replace(result[-1],heading_override_deg=part[0].heading_override_deg)
                result.extend(part[1:])
            else: result.extend(part)
        if not any(p.arc_source=='FREE_CURVE_FIT' for p in result[1:]):
            return self.reference_points(sa,sb)
        if depth==0:
            original_cost=self.original_block_cost(sa,sb)
            trial=Stroke(self.f.stroke.id,self.f.stroke.layer_id,True,result)
            trial=restore_fallback_orientation(trial,self)
            refined_cost=machine_block_cost(Field(trial,self.cfg,self.reference_c(sa)),self.cfg)
            if refined_cost>=original_cost:
                for log in self.logs[log_start:]: log['accepted']=False
                self.logs.append({'span_id':span_id,'source_point_count':len(self.reference_points(sa,sb)),
                                  'classification':'SMOOTH_CURVE','split_reason':'BLOCK_COST_NOT_IMPROVED',
                                  'output':'REFINEMENT_REJECTED','primitive_count':original_cost,'max_error':None,
                                  'refined_machine_blocks':refined_cost,'coarse_machine_blocks':original_cost})
                return self.reference_points(sa,sb)
        return result

    def fit_interval(self,start,end,span_id,depth=0,recognized_curve=False):
        f=self.f; pts=f.stroke.points
        if end-start+1<self.cfg.arc_fit.minimum_source_points and not recognized_curve: return pts[start:end+1]
        near_line=circular_candidate([p.xyz for p in pts[start:end+1]])
        if (near_line and abs(math.degrees(near_line.sweep))<=self.angle_tolerance and
                near_line.radius*(1-math.cos(near_line.sweep/2))<=self.geometry_tolerance):
            self.logs.append({'span_id':span_id,'source_point_count':end-start+1,'classification':'NEAR_STRAIGHT',
                              'split_reason':None,'output':'LINE','primitive_count':0,'max_error':None})
            return pts[start:end+1]
        candidates,reason,worst=self.candidates(start,end)
        # Extend prefixes geometrically, then refine the last valid endpoint.
        # Closed circular spans and sweep-limited regions retain quarter splits.
        if (not candidates and end-start>8 and depth<40 and reason!='SWEEP_LIMIT' and
                math.dist(pts[start].xyz,pts[end].xyz)>self.geometry_tolerance):
            best=start; probe=start+self.cfg.arc_fit.minimum_source_points-1; failed=end
            while probe<end:
                trial,_,_=self.candidates(start,probe)
                if not trial: failed=probe; break
                best=probe; probe=min(end,start+2*(probe-start))
            if best>start:
                lo,hi=best,failed
                while hi-lo>1:
                    middle=(lo+hi)//2
                    trial,_,_=self.candidates(start,middle)
                    if trial: lo=middle
                    else: hi=middle
                best=lo
                if best<end-1:
                    self.logs.append({'span_id':span_id,'source_point_count':end-start+1,'classification':'SMOOTH_CURVE',
                                      'split_reason':reason,'output':'PROGRESSIVE_SPLIT','primitive_count':0,'max_error':None})
                    left=self.fit_interval(start,best,span_id+'P',depth+1,True)
                    right=self.fit_interval(best,end,span_id+'R',depth+1,True)
                    if right[0].heading_override_deg is not None: left[-1]=replace(left[-1],heading_override_deg=right[0].heading_override_deg)
                    return left+right[1:]
        if candidates:
            return self.accepted_points(candidates,span_id)
        if end-start<=2 and self.cfg.arc_fit.adaptive_refinement and self.geometry_tolerance>=1e-4:
            refined=self.refine_span(f.s[start],f.s[end],span_id+'A')
            if any(p.arc_source=='FREE_CURVE_FIT' for p in refined[1:]):
                self.logs.append({'span_id':span_id,'source_point_count':end-start+1,'classification':'SMOOTH_CURVE',
                                  'split_reason':reason,'output':'ADAPTIVE_REFINEMENT','primitive_count':0,'max_error':None})
                return refined
        if end-start<=1 or depth>=40:
            self.logs.append({'span_id':span_id,'source_point_count':end-start+1,'classification':'SMOOTH_CURVE',
                              'split_reason':reason,'output':'G1_FALLBACK','primitive_count':end-start,'max_error':None})
            return pts[start:end+1]
        split=bisect.bisect_left(f.s,worst) if worst is not None else (start+end)//2
        split=max(start+1,min(end-1,split))
        self.logs.append({'span_id':span_id,'source_point_count':end-start+1,'classification':'SMOOTH_CURVE',
                          'split_reason':reason,'output':'SPLIT','primitive_count':0,'max_error':None})
        left=self.fit_interval(start,split,span_id+'L',depth+1,True)
        right=self.fit_interval(split,end,span_id+'R',depth+1,True)
        # The right start may carry an accepted tangent override for a shared boundary.
        left[-1]=replace(left[-1],heading_override_deg=right[0].heading_override_deg if right[0].heading_override_deg is not None else left[-1].heading_override_deg)
        return left+right[1:]


def merge_fitted_arcs(stroke,fitter,protected_s):
    """Coalesce co-circular neighbours only after revalidating the original span."""
    points=list(stroke.points); i=0
    while i+2<len(points):
        a,m,b=points[i:i+3]
        if (m.arc_source==b.arc_source=='FREE_CURVE_FIT' and m.feed==b.feed and
                m.arc_sweep_deg*b.arc_sweep_deg>0 and
                math.dist(m.arc_center,b.arc_center)<=fitter.geometry_tolerance and
                abs(m.arc_radius-b.arc_radius)<=fitter.cfg.arc_fit.radial_tolerance_mm):
            sa=m.fit_metadata['reference_s_start']; sb=b.fit_metadata['reference_s_end']
            if not any(sa+1e-8<s<sb-1e-8 for s in protected_s):
                candidate=circular_candidate([a.xyz,m.xyz,b.xyz])
                if candidate:
                    candidate.sa,candidate.sb=sa,sb
                    good,_,_=fitter.validate(candidate)
                    if good and candidate.metrics['max_tangent_error_deg']<1e-3:
                        metadata=dict(candidate.metrics,source_span=m.fit_metadata['source_span']+'+MERGED',
                                      source_point_count=len({*(m.source_lines or []),*(b.source_lines or [])}),
                                      reference_s_start=sa,reference_s_end=sb,
                                      candidate_c_start=nearest_equivalent_angle(candidate.heading+fitter.cfg.orientation_offset_deg,
                                                                                fitter.reference_c(sa),360.))
                        points[i]=replace(a,heading_override_deg=candidate.heading)
                        points[i+2]=replace(b,arc_center=candidate.center,arc_radius=candidate.radius,
                                            arc_sweep_deg=math.degrees(candidate.sweep),fit_metadata=metadata,
                                            heading_override_deg=candidate.heading+math.degrees(candidate.sweep),
                                            source_lines=sorted(set(m.source_lines+b.source_lines)))
                        points.pop(i+1)
                        fitter.logs.append({'span_id':metadata['source_span'],'source_point_count':metadata['source_point_count'],
                                            'classification':'SMOOTH_CURVE','split_reason':None,'output':'MERGED_ARC',
                                            'primitive_count':1,'max_error':metadata['max_fit_error_mm']})
                        continue
        i+=1
    return Stroke(stroke.id,stroke.layer_id,stroke.deposition,points,stroke.seam_reindexed)


def restore_fallback_orientation(stroke,fitter):
    """Retain the original C polynomial on unfitted portions of a mixed path.

    Recomputing PCHIP slopes after inserting arc/virtual endpoints would alter
    the old C(s) on neighbouring G1 blocks, even if their XYZ is identical.
    """
    f=fitter.f
    if not any(p.arc_source=='FREE_CURVE_FIT' for p in stroke.points): return stroke
    original={(p.motion_index,p.xyz):s for p,s in zip(f.stroke.points,f.s)}
    positions=[p.fit_metadata.get('reference_s',p.fit_metadata.get('reference_s_end',
                   original.get((p.motion_index,p.xyz)))) for p in stroke.points]
    points=list(stroke.points)
    for i,(sa,sb) in enumerate(zip(positions,positions[1:])):
        if points[i+1].arc_center is not None or sa is None or sb is None or sb-sa<=1e-9: continue
        first=fitter.location(sa)[0]; last=fitter.location(sb-1e-9)[0]
        if any(k+1 in f.by_vertex for k in range(first,last+1)): continue
        pieces=[]
        for k in range(first,last+1):
            h=f.seg_length[k]
            for lo,hi,coeff in f.polynomial_pieces(k):
                a=max(sa,f.s[k]+lo*h); b=min(sb,f.s[k]+hi*h)
                if b-a<=1e-12: continue
                pieces.append(((a-sa)/(sb-sa),(b-sa)/(sb-sa),
                               compose_affine(coeff,(sa-f.s[k])/h,(sb-sa)/h)))
        metadata=dict(points[i+1].fit_metadata,reference_s_start=sa,reference_s_end=sb,
                      reference_c_start=fitter.reference_c(sa),reference_c_end=fitter.reference_c(sb),
                      reference_orientation_pieces=pieces)
        points[i+1]=replace(points[i+1],fit_metadata=metadata)
        for j,s in ((i,sa),(i+1,sb)):
            if points[j].heading_override_deg is None:
                points[j]=replace(points[j],heading_override_deg=fitter.reference_c(s)-fitter.cfg.orientation_offset_deg)
    return Stroke(stroke.id,stroke.layer_id,stroke.deposition,points,stroke.seam_reindexed)


def machine_block_cost(field,config):
    """Count native primitives, excluding arc tessellation used by the viewer."""
    from .sampling import CurveSampler
    from .primitives import arc_parts
    rows,_=CurveSampler(field,config).sample()
    return sum(arc_parts(field.stroke.points[i+1]) if field.stroke.points[i+1].arc_center is not None
               else len(values) for i,values in rows.items())


def fit_stroke(stroke,config,previous_c=None,representation=None):
    if (not stroke.deposition or not config.native_arcs.enabled or not config.arc_fit.enabled or not config.native_arcs.fitted_free_curves.enabled or
            config.serialization.motion_mode=='G1_ONLY' or config.sampling.mode=='legacy' or len(stroke.points)<3):
        return stroke,[]
    if config.arc_fit.reference_model=='PARAMETRIC_HERMITE' and config.geometry.mode!='PRESERVE_PATH':
        from .curve_reconstruction import reconstruct_stroke
        return reconstruct_stroke(stroke,config,previous_c,representation)
    reference=Field(stroke,config,previous_c); fitter=ArcFitter(reference,config)
    pts=stroke.points; boundaries={0:None,len(pts)-1:None}
    for i,p in enumerate(pts):
        if p.locked or i in reference.by_vertex: boundaries[i]='LOCKED_EVENT'
        if p.arc_center is not None:
            boundaries[i]=boundaries[i-1]='LOCKED_EVENT'
        if i and (p.feed!=pts[i-1].feed or abs(p.xyz[2]-pts[i-1].xyz[2])>config.precision.geometry_epsilon):
            boundaries[i-1]='LOCKED_EVENT'
            if abs(p.xyz[2]-pts[i-1].xyz[2])>config.precision.geometry_epsilon: boundaries[i]='LOCKED_EVENT'
    last_sign=0
    for i in range(1,len(pts)-1):
        turn=wrap180(reference.seg_unwrapped[i]-reference.seg_unwrapped[i-1])
        sign=1 if turn>1e-6 else -1 if turn< -1e-6 else 0
        if sign and last_sign and sign!=last_sign: boundaries[i]='INFLECTION'
        if sign: last_sign=sign
    result=[]
    cuts=sorted(boundaries)
    for start,end in zip(cuts,cuts[1:]):
        span=f'{stroke.id}:{start}-{end}'
        if (any(p.arc_center for p in pts[start+1:end+1]) or start in reference.by_vertex or end in reference.by_vertex or
                any(p.xyz[2]!=pts[start].xyz[2] for p in pts[start:end+1])):
            part=pts[start:end+1]
        else: part=fitter.fit_interval(start,end,span)
        if boundaries.get(end):
            fitter.logs.append({'span_id':span,'source_point_count':end-start+1,'classification':'BOUNDARY',
                                'split_reason':boundaries[end],'output':'BOUNDARY','primitive_count':0,'max_error':None})
        if result:
            if part[0].heading_override_deg is not None:
                result[-1]=replace(result[-1],heading_override_deg=part[0].heading_override_deg)
            result.extend(part[1:])
        else: result.extend(part)
    fitted=Stroke(stroke.id,stroke.layer_id,stroke.deposition,result,stroke.seam_reindexed)
    fitted=merge_fitted_arcs(fitted,fitter,[reference.s[cut] for cut in cuts])
    fitted=restore_fallback_orientation(fitted,fitter)
    final_field=Field(fitted,config,previous_c)
    for i,p in enumerate(fitted.points[1:]):
        if p.arc_source!='FREE_CURVE_FIT':
            if 'reference_orientation_pieces' not in p.fit_metadata: continue
            error=max(abs(final_field.point_c[i]-p.fit_metadata['reference_c_start']),
                      abs(final_field.point_c[i+1]-p.fit_metadata['reference_c_end']))
        else:
            metadata=p.fit_metadata
            shift=final_field.point_c[i]-metadata['candidate_c_start']
            error=metadata['max_orientation_fit_error_deg']+abs(shift)
        if error>fitter.angle_tolerance+1e-8:
            for log in fitter.logs:
                if log['output'] in ('ARC','BIARC','MERGED_ARC'): log['accepted']=False
            fitter.logs.append({'span_id':f'{stroke.id}:FINAL_C_VALIDATION','source_point_count':len(pts),
                                'classification':'SMOOTH_CURVE','split_reason':'ARC_JOIN_TANGENT_DISCONTINUITY',
                                'output':'G1_FALLBACK','primitive_count':len(pts)-1,'max_error':error})
            return stroke,fitter.logs
        if p.arc_source=='FREE_CURVE_FIT': metadata['max_orientation_fit_error_deg']=error
    if config.arc_fit.adaptive_refinement and any(d['output']=='ADAPTIVE_REFINEMENT' for d in fitter.logs):
        # Refinement is useful only if it avoids fragmentation. Compare to the
        # same improved fitter without virtual boundaries, not to display dots.
        coarse_config=replace(config,arc_fit=replace(config.arc_fit,adaptive_refinement=False))
        coarse,coarse_logs=fit_stroke(stroke,coarse_config,previous_c)
        fine_cost=machine_block_cost(final_field,config)
        coarse_cost=machine_block_cost(Field(coarse,config,previous_c),config)
        if fine_cost>=coarse_cost:
            coarse_logs.append({'span_id':f'{stroke.id}:REFINEMENT_COST','source_point_count':len(pts),
                                'classification':'SMOOTH_CURVE','split_reason':'BLOCK_COST_NOT_IMPROVED',
                                'output':'COARSE_FIT_SELECTED','primitive_count':coarse_cost,'max_error':None,
                                'refined_machine_blocks':fine_cost,'coarse_machine_blocks':coarse_cost})
            return coarse,coarse_logs
    return fitted,fitter.logs
