"""Optional cyclic start selection; coordinates are never translated."""
from dataclasses import replace
import math
from .path import Stroke, xy_length
from .classification import classify_vertices


def plan_seam(stroke, fillets, config):
    report={'stroke_id':stroke.id,'layer_id':stroke.layer_id,'status':'NOT CONFIGURED',
            'enabled':config.layer_transition.safe_seam,'selected_xyz':list(stroke.points[0].xyz),
            'corner_guard_mm':config.layer_transition.corner_guard_mm,'corner_distance_mm':None,
            'reindexed':False,'reason':'Selección de seam desactivada'}
    if not stroke.deposition or not config.layer_transition.safe_seam:
        return stroke,report
    pts=stroke.points
    if len(pts)<4 or math.dist(pts[0].xyz,pts[-1].xyz)>config.geometry.zero_length_tolerance_mm:
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
    for i,(a,b) in enumerate(zip(pts,pts[1:])):
        length=s[i+1]-s[i]
        if b.arc_center or length < max(config.geometry.tiny_segment_warning_mm,2*config.layer_transition.corner_guard_mm):
            continue
        # Neighbor headings must be collinear or pinned to an analytic tangent.
        heading=math.degrees(math.atan2(b.xyz[1]-a.xyz[1],b.xyz[0]-a.xyz[0]))
        def straight_endpoint(index):
            p=pts[index]
            if p.heading_override_deg is not None:
                return abs((p.heading_override_deg-heading+180)%360-180)<config.precision.angle_epsilon_deg
            return cls.get(index,{}).get('category','LINE')!='SMOOTH_CURVE'
        if not straight_endpoint(i) or not straight_endpoint(i+1):
            continue
        midpoint=(s[i]+s[i+1])/2
        distance=min((min(abs(midpoint-p),total-abs(midpoint-p)) for p in guard_positions),default=total/2)
        if distance+config.precision.geometry_epsilon<config.layer_transition.corner_guard_mm:
            continue
        # Zero curvature and dC/ds here. Long physical clearance is the remaining score.
        score=1/max(distance,1e-9)+1/max(length,1e-9)
        candidates.append((score,i,distance,heading))
    if not candidates:
        report.update(status='WARNING',reason='No existe recta con guard físico suficiente')
        return stroke,report
    score,i,distance,heading=min(candidates)
    a,b=pts[i:i+2]
    xyz=tuple((x+y)/2 for x,y in zip(a.xyz,b.xyz))
    seam=replace(b,xyz=xyz,fillet_id=None,generated_by_fillet=False,heading_override_deg=heading,
                 arc_center=None,arc_radius=None,arc_sweep_deg=None)
    ordered=[replace(seam)]+[replace(p) for p in pts[i+1:]]+[replace(p) for p in pts[1:i+1]]+[replace(seam)]
    for f in fillets:
        if f.status=='PASS':
            f.optimized_point_start=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t1)<1e-7)
            f.optimized_point_end=next(j for j,p in enumerate(ordered) if math.dist(p.xyz,f.t2)<1e-7)
    report.update(status='PASS',selected_xyz=list(xyz),corner_distance_mm=distance,reindexed=True,
                  source_line=b.line,score=score,reason='Reindexado cíclico en recta; sin traslación XY')
    guard=config.layer_transition.corner_guard_mm
    tangent=(math.cos(math.radians(heading)),math.sin(math.radians(heading)))
    report['guard_start_xyz']=[xyz[0]-guard*tangent[0],xyz[1]-guard*tangent[1],xyz[2]]
    report['guard_end_xyz']=[xyz[0]+guard*tangent[0],xyz[1]+guard*tangent[1],xyz[2]]
    return Stroke(stroke.id,stroke.layer_id,stroke.deposition,ordered,True),report
