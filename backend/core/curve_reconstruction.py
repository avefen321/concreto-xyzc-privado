"""Arc splines fitted to one bounded parametric XYZ/tangent reference.

On-curve biarc joins follow the joint-circle construction described by
Sir/Feichtinger/Juettler (2006). This is an independent implementation; no
external library code is included. Polynomial extrema certify the radial and
tangent-cone inequalities on complete Bezier intervals, not only vertices.
"""
from dataclasses import replace
import math
from .arc_fitting import FittedArc,tangent_arc
from .orientation import Field,wrap180,nearest_equivalent_angle
from .parametric import (ParametricReference,coefficients,poly_add,poly_mul,polynomial_roots,
                         polynomial_range,restricted_controls,split_controls)
from .numerics import derivative,value
from .path import Stroke
from .simplification import distance_to_chord


MODEL='PARAMETRIC_HERMITE'


class ReconstructionFitter:
    def __init__(self,reference,config):
        self.ref=reference; self.config=config; self.logs=[]
        self.last_reject=None
        self.geometry=min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm,
                          config.simplification.geometry_tolerance_mm)
        self.rounding_budget=2*math.sqrt(2)*10**(-config.serialization.xyz_decimals)+2*10**(-config.serialization.ijk_decimals)
        self.budget=self.geometry-reference.source_error_bound-self.rounding_budget
        if self.budget<=0: raise ValueError('Precisión XYZ/IJ insuficiente para el presupuesto geométrico de reconstrucción')
        self.angle=.8*min(config.arc_fit.orientation_tolerance_deg,config.arc_fit.tangent_tolerance_deg,
                         config.sampling.max_orientation_interp_error_deg,config.simplification.orientation_tolerance_deg)

    def tangent_error_bound(self,controls,center,direction):
        """tan(error)= -dot(B-O,B') / cross(B-O,B'); certify a forward cone."""
        maximum=0.; stack=[(controls,0)]
        while stack:
            p,depth=stack.pop(); x,y=coefficients(p,center); dx,dy=derivative(x),derivative(y)
            cross=poly_add(poly_mul(x,dy),poly_mul(y,dx),-1.)
            dot=poly_add(poly_mul(x,dx),poly_mul(y,dy))
            forward=[direction*v for v in cross]
            lo,_=polynomial_range(forward); emin,emax=polynomial_range(dot)
            bound=math.degrees(math.atan2(max(abs(emin),abs(emax)),lo)) if lo>0 else 180.
            if bound<=self.angle:
                maximum=max(maximum,bound); continue
            for t in (0.,.5,1.):
                if value(forward,t)<=0 or abs(math.degrees(math.atan2(value(dot,t),value(forward,t))))>self.angle+1e-9:
                    return None
            if depth>=10: return None
            left,right=split_controls(p,.5); stack.extend(((left,depth+1),(right,depth+1)))
        return maximum

    def validate_arc(self,arc):
        cfg=self.config.arc_fit
        def fail(reason):
            self.last_reject=reason
            return False,reason
        if (not all(math.isfinite(v) for v in (*arc.center,arc.radius,arc.sweep)) or
                arc.radius<cfg.min_radius_mm or cfg.max_radius_mm and arc.radius>cfg.max_radius_mm): return fail('RADIUS_LIMIT')
        if abs(math.degrees(arc.sweep))>cfg.max_sweep_deg+1e-8: return fail('SWEEP_LIMIT')
        direction=math.copysign(1,arc.sweep); radial=angular=source=0.
        phase=0.; previous=math.atan2(arc.start[1]-arc.center[1],arc.start[0]-arc.center[0])
        for piece,p,a,b in self.ref.portions(arc.sa,arc.sb):
            x,y=coefficients(p,arc.center)
            radius2=poly_add(poly_mul(x,x),poly_mul(y,y)); lo,hi=polynomial_range(radius2)
            error=max(abs(math.sqrt(max(0.,lo))-arc.radius),abs(math.sqrt(max(0.,hi))-arc.radius))
            radial=max(radial,error); source=max(source,piece.source_error_bound)
            if radial>min(self.budget,cfg.radial_tolerance_mm): return fail('FIT_ERROR')
            angle=self.tangent_error_bound(p,arc.center,direction)
            if angle is None: return fail('TANGENT_ERROR')
            angular=max(angular,angle)
            # The positive cross-product certificate establishes monotone polar
            # order. Midpoints disambiguate the atan2 branch at piece joins.
            for point in (split_controls(p,.5)[0][-1],p[-1]):
                phi=math.atan2(point[1]-arc.center[1],point[0]-arc.center[0])
                phase+=math.radians(wrap180(math.degrees(phi-previous))); previous=phi
        if abs(phase-arc.sweep)>getattr(self,'phase_tolerance',1e-7): return fail('POINT_ORDER')
        start_error=abs(wrap180(arc.heading-self.ref.heading(arc.sa)))
        end_error=abs(wrap180(arc.heading+math.degrees(arc.sweep)-self.ref.heading(arc.sb)))
        if max(start_error,end_error)>self.angle+1e-8: return fail('TANGENT_ERROR')
        arc.metrics={'max_fit_error_mm':source+radial,'max_radial_error_mm':radial,
                     'output_rounding_geometry_budget_mm':self.rounding_budget,
                     'reconstruction_error_mm':source,'approximation_error_mm':radial,
                     'max_orientation_fit_error_deg':max(angular,start_error,end_error),
                     'max_tangent_error_deg':max(start_error,end_error),'reference_model':MODEL,
                     'certificate':'POLYNOMIAL_RADIAL_AND_TANGENT_CONE'}
        return True,None

    def on_curve_biarcs(self,a,b):
        ref=self.ref; p0,p1=ref.xyz(a),ref.xyz(b)
        v=(p1[0]-p0[0],p1[1]-p0[1]); length=math.hypot(*v)
        if length<1e-8: return []
        h0,h1=ref.heading(a),ref.heading(b)
        delta=math.radians(h1-h0)/2; sd,cd=math.sin(delta),math.cos(delta)
        joins=[]
        for piece,p,lo,hi in ref.portions(a,b):
            x,y=coefficients(p,p0[:2])
            # Joint-circle equation, kept implicit to avoid cot(delta) near 0.
            f=poly_add([sd*z for z in poly_add(poly_mul(x,x),poly_mul(y,y))],
                       [(-sd*v[0]+cd*v[1])*z for z in x])
            f=poly_add(f,[(-sd*v[1]-cd*v[0])*z for z in y])
            scale=max(map(abs,f),default=0.)
            ts=polynomial_roots(f) if scale>1e-13*max(length*length,1.) else [.5]
            for t in ts:
                u=lo+(hi-lo)*t
                if a+(b-a)*1e-6<u<b-(b-a)*1e-6: joins.append(u)
        for middle in sorted(set(joins),key=lambda u:abs(u-(a+b)/2))[:7]:
            q=ref.xyz(middle)
            t0=(math.cos(math.radians(h0)),math.sin(math.radians(h0)))
            t1=(math.cos(math.radians(h1)),math.sin(math.radians(h1)))
            first=tangent_arc(p0,q,t0); reverse=tangent_arc(p1,q,(-t1[0],-t1[1]))
            if first is None or reverse is None: continue
            second=FittedArc(q,p1,reverse.center,reverse.radius,-reverse.sweep,middle,b)
            first.sa,first.sb=a,middle
            if abs(wrap180(first.heading+math.degrees(first.sweep)-second.heading))>1e-6: continue
            if first.sweep*second.sweep<=0: continue
            if self.validate_arc(first)[0] and self.validate_arc(second)[0]: return [first,second]
        return []

    def line_orientation_bound(self,a,b,start,end,heading_start=None,heading_end=None):
        ref=self.ref
        h0=ref.heading(a) if heading_start is None else heading_start
        change=(ref.heading(b) if heading_end is None else heading_end)-h0
        chord=(end[0]-start[0],end[1]-start[1]); length2=sum(x*x for x in chord)
        if length2<1e-18: return None
        maximum=0.
        for piece,controls,lo,hi in ref.portions(a,b):
            stack=[(controls,lo,hi,0)]
            while stack:
                p,left,right,depth=stack.pop(); x,y=coefficients(p,start[:2])
                q=poly_add([chord[0]*v/length2 for v in x],[chord[1]*v/length2 for v in y])
                target=[h0+change*q[0]]+[change*v for v in q[1:]]
                cmin,cmax=polynomial_range(target)
                anchor=ref.heading((left+right)/2)
                headings=[nearest_equivalent_angle(math.degrees(math.atan2(v[1]-u[1],v[0]-u[0])),anchor,360.)
                          for u,v in zip(p,p[1:]) if math.dist(u[:2],v[:2])>1e-12]
                if not headings: return None
                bound=max(abs(min(headings)-cmax),abs(max(headings)-cmin))
                if bound<=self.angle: maximum=max(maximum,bound); continue
                if abs(ref.heading((left+right)/2)-value(target,.5))>self.angle+1e-8: return None
                if depth>=10: return None
                lp,rp=split_controls(p,.5); middle=(left+right)/2
                stack.extend(((lp,left,middle,depth+1),(rp,middle,right,depth+1)))
        return maximum

    def line_candidate(self,a,b):
        ref=self.ref; start,end=ref.xyz(a),ref.xyz(b)
        if math.dist(start[:2],end[:2])<1e-9: return None
        chord_heading=math.degrees(math.atan2(end[1]-start[1],end[0]-start[0]))
        pose_error=max(abs(wrap180(ref.heading(u)-chord_heading)) for u in (a,b))
        if pose_error>self.angle: return None
        precision=self.config.serialization
        rounded_a=tuple(round(v,precision.xyz_decimals) for v in start)
        rounded_b=tuple(round(v,precision.xyz_decimals) for v in end)
        if math.dist(rounded_a[:2],rounded_b[:2])<1e-12: return None
        exported_heading=math.degrees(math.atan2(rounded_b[1]-rounded_a[1],rounded_b[0]-rounded_a[0]))
        exported_pose=max(abs(wrap180(round(ref.heading(u)+self.config.orientation_offset_deg,precision.c_decimals)-
                                     self.config.orientation_offset_deg-exported_heading)) for u in (a,b))
        if exported_pose>self.angle: return None
        error=source=0.
        for piece,p,lo,hi in ref.portions(a,b):
            error=max(error,max(distance_to_chord(q,start,end) for q in p))
            source=max(source,piece.source_error_bound)
            if error>self.budget: return None
        angle=self.line_orientation_bound(a,b,start,end)
        if angle is None: return None
        return {'max_fit_error_mm':source+error,'reconstruction_error_mm':source,
                'approximation_error_mm':error,'max_orientation_fit_error_deg':angle,
                'max_line_pose_error_deg':pose_error,
                'max_exported_line_pose_error_deg':exported_pose,
                'output_rounding_geometry_budget_mm':self.rounding_budget,
                'reference_model':MODEL,'certificate':'BEZIER_HULL_AND_TANGENT_CONE',
                'reference_c_start':ref.heading(a)+self.config.orientation_offset_deg,
                'reference_c_end':ref.heading(b)+self.config.orientation_offset_deg,
                'reference_orientation_pieces':[(0.,1.,[ref.heading(a)+self.config.orientation_offset_deg,
                                                       ref.heading(b)-ref.heading(a)])]}

    def point(self,u):
        i,t=self.ref.location(u); piece=self.ref.pieces[i]
        index=piece.source_index if t<1e-10 else piece.source_index+1
        p=self.ref.stroke.points[index]
        return replace(p,xyz=self.ref.xyz(u),heading_override_deg=self.ref.heading(u),
                       arc_center=None,arc_radius=None,arc_sweep_deg=None,arc_source=None,
                       locked=True,fit_metadata={},source_lines=[p.line])

    def materialize(self,a,b,arcs=None,line=None):
        ref=self.ref; result=[self.point(a)]
        if arcs:
            for arc in arcs:
                p=self.point(arc.sb)
                meta=dict(arc.metrics,reference_u_start=arc.sa,reference_u_end=arc.sb,
                          candidate_c_start=arc.heading+self.config.orientation_offset_deg)
                result.append(replace(p,xyz=arc.end,heading_override_deg=arc.heading+math.degrees(arc.sweep),
                                      arc_center=arc.center,arc_radius=arc.radius,arc_sweep_deg=math.degrees(arc.sweep),
                                      arc_source='FREE_CURVE_FIT',fit_metadata=meta))
        else:
            result.append(replace(self.point(b),arc_source='FREE_CURVE_FIT',
                                  fit_metadata=dict(line,reference_u_start=a,reference_u_end=b)))
        for point in result[1:]:
            first=self.ref.location(point.fit_metadata['reference_u_start'])[0]
            last=self.ref.location(point.fit_metadata['reference_u_end']-1e-9)[0]
            point.source_lines=sorted({line for piece in ref.pieces[first:last+1]
                                      for line in (ref.stroke.points[piece.source_index+1].source_lines or
                                                   [ref.stroke.points[piece.source_index+1].line])})
        return result

    def fit(self,a,b,depth=0):
        ref=self.ref; self.last_reject=None
        # Flat regions remain efficient G1, with the same geometric/angle audit.
        line=self.line_candidate(a,b)
        if line and abs(ref.heading(b)-ref.heading(a))<=self.angle:
            line['fallback_reason']='NEAR_STRAIGHT'
            return self.materialize(a,b,line=line)
        p0,p1=ref.xyz(a),ref.xyz(b); h0=math.radians(ref.heading(a))
        arc=tangent_arc(p0,p1,(math.cos(h0),math.sin(h0)))
        if arc:
            arc.sa,arc.sb=a,b
            if abs(wrap180(arc.heading+math.degrees(arc.sweep)-ref.heading(b)))<1e-9 and self.validate_arc(arc)[0]:
                return self.materialize(a,b,arcs=[arc])
        if self.config.arc_fit.biarc_enabled:
            arcs=self.on_curve_biarcs(a,b)
            if arcs: return self.materialize(a,b,arcs=arcs)
        if line:
            line['fallback_reason']=self.last_reject or 'NO_ADMISSIBLE_ON_CURVE_BIARC'
            return self.materialize(a,b,line=line)
        if depth>=18 or b-a<1e-9: raise ValueError('No se pudo certificar el tramo paramétrico')
        middle=(a+b)/2
        left=self.fit(a,middle,depth+1); right=self.fit(middle,b,depth+1)
        left[-1]=replace(left[-1],heading_override_deg=right[0].heading_override_deg)
        if len(left)+len(right)>self.config.sampling.max_samples_per_stroke:
            raise ValueError('max_samples_per_stroke superado por reconstrucción')
        return left+right[1:]


def reconstruct_stroke(stroke,config,previous_c=None,representation=None):
    field=Field(stroke,config,previous_c); n=len(stroke.points)-1; eligible=[]
    for i,(a,b) in enumerate(zip(stroke.points,stroke.points[1:])):
        eligible.append(not (a.locked or b.locked or a.arc_center is not None or b.arc_center is not None or
                              i in field.by_vertex or i+1 in field.by_vertex or a.feed!=b.feed or a.xyz[2]!=b.xyz[2]))
    points=[]; logs=[]; i=0
    geometry=min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm,
                 config.simplification.geometry_tolerance_mm)
    def fit_span(start,end,depth=0):
        """Keep a singular subspan in G1 without discarding its smooth neighbours."""
        try:
            reference=ParametricReference(stroke,field,start,end,geometry*config.arc_fit.reconstruction_budget_fraction)
            reference_id=None
            if representation is not None:
                reference,reference_id=representation.reference(reference,stroke,start,end)
            fitter=ReconstructionFitter(reference,config); part=[]
            cuts=reference.cuts()
            for a,b in zip(cuts,cuts[1:]):
                interval=(representation.fit(fitter,a,b,reference_id)
                          if representation is not None else fitter.fit(a,b))
                if part:
                    part[-1]=replace(part[-1],heading_override_deg=interval[0].heading_override_deg)
                    part.extend(interval[1:])
                else: part.extend(interval)
            for left,right in zip(part,part[1:]):
                length=(abs(math.radians(right.arc_sweep_deg))*right.arc_radius
                        if right.arc_center is not None else math.dist(left.xyz[:2],right.xyz[:2]))
                if length<=config.geometry.zero_length_tolerance_mm:
                    raise ValueError('DEGENERATE_PARAMETRIC_CANDIDATE: se conserva el tramo original')
            arcs=sum(p.arc_center is not None for p in part[1:])
            if representation is not None:
                representation.accept(reference_id,part)
            reasons={}
            for p in part[1:]:
                if p.arc_center is None:
                    reason=p.fit_metadata.get('fallback_reason','UNKNOWN')
                    reasons[reason]=reasons.get(reason,0)+1
            return part,[{'span_id':f'{stroke.id}:{start}-{end}','classification':'PARAMETRIC_CURVE',
                          'source_point_count':end-start+1,'split_reason':None,'output':'PARAMETRIC_SPLINE',
                          'primitive_count':len(part)-1,'arc_count':arcs,'line_count':len(part)-1-arcs,
                          'reference_model':MODEL,'closed_periodic':reference.closed,
                          'fallback_reasons':reasons,
                          'reconstruction_bound_mm':reference.source_error_bound,'inflection_cuts':len(cuts)-2,
                          'max_error':max(p.fit_metadata['max_fit_error_mm'] for p in part[1:])}]
        except ValueError as error:
            reason=str(error)
            if (reason=='No se pudo certificar el tramo paramétrico' and depth<12 and
                    end-start>=2*config.arc_fit.minimum_source_points):
                middle=(start+end)//2
                left,left_logs=fit_span(start,middle,depth+1)
                right,right_logs=fit_span(middle,end,depth+1)
                if right[0].heading_override_deg is not None:
                    left[-1]=replace(left[-1],heading_override_deg=right[0].heading_override_deg)
                return left+right[1:],left_logs+right_logs
            return stroke.points[start:end+1],[{'span_id':f'{stroke.id}:{start}-{end}',
                            'classification':'PARAMETRIC_CURVE','source_point_count':end-start+1,
                            'split_reason':reason,'output':'G1_FALLBACK',
                            'primitive_count':end-start,'max_error':None,'reference_model':MODEL}]
    while i<n:
        end=i+1
        if eligible[i]:
            while end<n and eligible[end]: end+=1
        if not eligible[i] or end-i+1<config.arc_fit.minimum_source_points:
            part=stroke.points[i:end+1]
        else:
            part,part_logs=fit_span(i,end)
            logs.extend(part_logs)
        if points:
            if part[0].heading_override_deg is not None:
                points[-1]=replace(points[-1],heading_override_deg=part[0].heading_override_deg)
            points.extend(part[1:])
        else: points.extend(part)
        i=end
    fitted=Stroke(stroke.id,stroke.layer_id,stroke.deposition,points,stroke.seam_reindexed)
    final=Field(fitted,config,previous_c)
    for i,p in enumerate(points[1:]):
        if p.fit_metadata.get('reference_model')!=MODEL: continue
        start=p.fit_metadata.get('candidate_c_start',p.fit_metadata.get('reference_c_start'))
        if abs(wrap180(final.point_c[i]-start))>1e-5:
            logs.append({'span_id':f'{stroke.id}:FINAL_C','classification':'PARAMETRIC_CURVE','source_point_count':n+1,
                         'split_reason':'ARC_JOIN_TANGENT_DISCONTINUITY','output':'G1_FALLBACK',
                         'primitive_count':n,'max_error':None,'reference_model':MODEL,
                         'point_index':i,'actual_c':final.point_c[i],'candidate_c':start})
            return stroke,logs
    return fitted,logs
