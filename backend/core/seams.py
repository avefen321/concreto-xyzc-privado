"""Optional cyclic start selection; coordinates are never translated."""
from dataclasses import replace
import math
from .path import Stroke, xy_length
from .classification import classify_vertices


def plan_seam(stroke, fillets, config):
    report={'stroke_id':stroke.id,'layer_id':stroke.layer_id,'status':'NOT CONFIGURED',
            'enabled':config.layer_transition.safe_seam,'selected_xyz':list(stroke.points[0].xyz),
            'corner_guard_mm':config.layer_transition.corner_guard_mm,
            'preferred_corner_guard_mm':max(config.layer_transition.corner_guard_mm,
                                            config.layer_transition.preferred_corner_guard_mm),
            'preferred_guard_met':False,'corner_distance_mm':None,
            'reindexed':False,'reason':'Selección de seam desactivada'}
    if not stroke.deposition or not config.layer_transition.safe_seam:
        return stroke,report
    pts=stroke.points
    if len(pts)<2 or math.dist(pts[0].xyz,pts[-1].xyz)>config.geometry.zero_length_tolerance_mm:
        report.update(status='NOT APPLICABLE',reason='Trazo abierto')
        return stroke,report
    if len({p.feed for p in pts[1:]})>1 or max(p.xyz[2] for p in pts)-min(p.xyz[2] for p in pts)>config.layers.z_change_tolerance_mm:
        report.update(status='WARNING',reason='Se conserva el orden: feed variable o loop no plano')
        return stroke,report
    s=[0.]
    for a,b in zip(pts,pts[1:]):
        s.append(s[-1]+(abs(math.radians(b.arc_sweep_deg))*b.arc_radius if b.arc_center else xy_length(a.xyz,b.xyz)))
    total=s[-1]
    cls=classify_vertices(stroke,config)
    guard_positions=[s[i] for i,v in cls.items() if v['category'] in ('SHARP_CORNER','MODERATE_CORNER')]
    for i,p in enumerate(pts):
        if p.fillet_id is not None:
            guard_positions.append(s[i])
    candidates=[]
    straight_segments_seen=0
    for i,(a,b) in enumerate(zip(pts,pts[1:])):
        length=s[i+1]-s[i]
        if b.arc_center or length < config.geometry.tiny_segment_warning_mm:
            continue
        # Neighbor headings must be collinear or pinned to an analytic tangent.
        heading=math.degrees(math.atan2(b.xyz[1]-a.xyz[1],b.xyz[0]-a.xyz[0]))
        def straight_endpoint(index):
            p=pts[index]
            if p.heading_override_deg is not None:
                # Rounded source G2/G3 endpoints can differ from their exact
                # tangent by a few ten-thousandths of a degree. They are still
                # a safe straight tangent, unlike a real corner.
                return abs((p.heading_override_deg-heading+180)%360-180)<=0.25
            return cls.get(index,{}).get('category','LINE')!='SMOOTH_CURVE'
        if not straight_endpoint(i) or not straight_endpoint(i+1):
            continue
        straight_segments_seen+=1
        if length+config.precision.geometry_epsilon<2*config.layer_transition.corner_guard_mm:
            continue
        # Start at the physical midpoint of a straight segment. This gives
        # clearance on both sides and avoids starting next to a fillet.
        minimum=config.layer_transition.corner_guard_mm
        preferred=report['preferred_corner_guard_mm']
        t=0.5
        candidate_position=s[i]+t*length
        distance=min((min(abs(candidate_position-p),total-abs(candidate_position-p))
                      for p in guard_positions),default=total/2)
        if distance+config.precision.geometry_epsilon<minimum:
            continue
        xyz=tuple((a.xyz[j]+b.xyz[j])/2 for j in range(3))
        preferred_met=distance+config.precision.geometry_epsilon>=preferred
        # Prefer the requested clearance, then the midpoint nearest origin.
        score=math.hypot(xyz[0],xyz[1])
        candidates.append((not preferred_met,score,-distance,i,distance,heading,xyz))
    if not candidates:
        if straight_segments_seen==0:
            def tangent(a,b,point):
                if b.arc_center is not None:
                    radial=math.degrees(math.atan2(point.xyz[1]-b.arc_center[1],
                                                   point.xyz[0]-b.arc_center[0]))
                    return radial+(90 if b.arc_sweep_deg>0 else -90)
                return math.degrees(math.atan2(b.xyz[1]-a.xyz[1],b.xyz[0]-a.xyz[0]))
            curved=[]
            for i,p in enumerate(pts[:-1]):
                previous=pts[i-1] if i else pts[-2]
                incoming=pts[i] if i else pts[-1]
                outgoing=pts[i+1]
                before=tangent(previous,incoming,p)
                after=tangent(p,outgoing,p)
                turn=abs((after-before+180)%360-180)
                curved.append((turn>config.corner_detection.smooth_turn_max_deg,
                               math.hypot(p.xyz[0],p.xyz[1]),turn,i))
            if curved:
                _,_,turn,i=min(curved)
                xyz=pts[i].xyz
                ordered=([replace(pts[i],arc_center=None,arc_radius=None,arc_sweep_deg=None)]
                         +[replace(p) for p in pts[i+1:]]
                         +[replace(p) for p in pts[1:i+1]])
                for f in fillets:
                    if f.status=='PASS':
                        f.optimized_point_start=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t1)<1e-7)
                        f.optimized_point_end=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t2)<1e-7)
                report.update(status='WARNING',selected_xyz=list(xyz),reindexed=i!=0,
                              curve_start=True,curve_tangent_jump_deg=turn,
                              source_line=pts[i].line,
                              reason='Contorno sin recta segura: inicio en el punto de curva disponible más cercano a (0,0)')
                return Stroke(stroke.id,stroke.layer_id,stroke.deposition,ordered,i!=0),report
        report.update(status='WARNING',reason='No existe recta con guard físico suficiente')
        return stroke,report
    fallback,score,_,i,distance,heading,xyz=min(candidates)
    a,b=pts[i:i+2]
    seam=replace(b,xyz=xyz,fillet_id=None,generated_by_fillet=False,heading_override_deg=heading,
                 arc_center=None,arc_radius=None,arc_sweep_deg=None)
    ordered=[replace(seam)]+[replace(p) for p in pts[i+1:]]+[replace(p) for p in pts[1:i+1]]+[replace(seam)]
    for f in fillets:
        if f.status=='PASS':
            f.optimized_point_start=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t1)<1e-7)
            f.optimized_point_end=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t2)<1e-7)
    report.update(status='WARNING' if fallback else 'PASS',selected_xyz=list(xyz),
                  corner_distance_mm=distance,preferred_guard_met=not fallback,reindexed=True,
                  source_line=b.line,score=score,
                  reason=('No había recta con la distancia preferida; se usó el mínimo seguro'
                          if fallback else 'Reindexado cíclico en recta; sin traslación XY'))
    guard=min(distance,report['preferred_corner_guard_mm'])
    tangent=(math.cos(math.radians(heading)),math.sin(math.radians(heading)))
    report['guard_start_xyz']=[xyz[0]-guard*tangent[0],xyz[1]-guard*tangent[1],xyz[2]]
    report['guard_end_xyz']=[xyz[0]+guard*tangent[0],xyz[1]+guard*tangent[1],xyz[2]]
    return Stroke(stroke.id,stroke.layer_id,stroke.deposition,ordered,True),report
