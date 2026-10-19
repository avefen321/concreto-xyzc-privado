"""Local geometric evidence shared by the existing fillet and tangent planners."""
import math
from .path import xy_length


def classify_vertices(stroke, config):
    points = [p.xyz for p in stroke.points]
    n = len(points)
    closed = n > 3 and xy_length(points[0], points[-1]) <= config.geometry.zero_length_tolerance_mm
    result = {}
    for i in range(1, n if closed else n-1):
        a,b,c=points[i-1],points[i],points[1] if i==n-1 else points[i+1]
        l,r=xy_length(a,b),xy_length(b,c)
        if min(l,r)<=config.geometry.zero_length_tolerance_mm:
            continue
        ux,uy=(b[0]-a[0])/l,(b[1]-a[1])/l
        vx,vy=(c[0]-b[0])/r,(c[1]-b[1])/r
        turn=math.degrees(math.atan2(ux*vy-uy*vx,ux*vx+uy*vy))
        result[i]={'point_index':i,'turn_deg':turn,'curvature':math.radians(turn)/((l+r)/2),
                   'segment_length_mm':min(l,r),'category':'SHARP_CORNER'}
    for i,item in result.items():
        turn=abs(item['turn_deg'])
        if turn <= config.precision.angle_epsilon_deg:
            category='LINE'
        elif turn <= config.corner_detection.smooth_turn_max_deg:
            category='SMOOTH_CURVE'
        else:
            neighbors=[result[j]['curvature'] for j in (i-1,i+1) if j in result]
            k=item['curvature']
            coherent=(neighbors and all(k*v>0 and abs(k-v)<=config.corner_detection.curvature_relative_tolerance*max(abs(k),abs(v)) for v in neighbors))
            # A short chain of consistently turning chords can have strongly
            # varying curvature (ellipse/sine or coarse rounded polygon). Its
            # evidence differs from an isolated turn between two straight runs.
            if config.sampling.mode!='legacy':
                adjacent=[result[j] for j in (i-1,i+1) if j in result]
                supported=any(item['turn_deg']*v['turn_deg']>0 and
                              .25<=abs(v['turn_deg']/item['turn_deg'])<=4
                              for v in adjacent)
                coherent=coherent or supported
            category=('SMOOTH_CURVE' if coherent and turn < config.corner_detection.sharp_corner_min_deg
                      and len(result)>=3 else 'SHARP_CORNER' if turn>=config.corner_detection.sharp_corner_min_deg else 'MODERATE_CORNER')
        item['category']=category
    return result
