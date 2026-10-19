"""Arc-length tangents, stroke-local corner planning, and C(s)."""
from dataclasses import dataclass, asdict
import math
from .config import Config
from .path import Stroke, xy_length
from .classification import classify_vertices
from .numerics import compose_affine, max_abs, derivative


def wrap180(angle):
    return (angle + 180) % 360 - 180 if (angle + 180) % 360 else 180.0


def nearest_equivalent_angle(target, previous, period):
    return target + round((previous - target) / period) * period


def nozzle_angle(heading, config):
    """Canonical tangent -> nozzle vector -> calibrated, continuous angular target."""
    theta=math.radians(heading)
    tx,ty=math.cos(theta),math.sin(theta)
    alignment=config.orientation_offset_deg-config.orientation.calibration_offset_deg
    if alignment==90: nx,ny=-ty,tx
    elif alignment==-90: nx,ny=ty,-tx
    else:
        a=math.radians(alignment)
        nx,ny=tx*math.cos(a)-ty*math.sin(a),tx*math.sin(a)+ty*math.cos(a)
    raw=math.degrees(math.atan2(ny,nx))+config.orientation.calibration_offset_deg
    return nearest_equivalent_angle(raw,heading+config.orientation_offset_deg,360)


def quintic(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (10 + u * (-15 + 6 * u))


@dataclass
class Corner:
    id: int
    stroke_id: int
    layer_id: int | None
    point_index: int
    source_line: int
    s_vertex: float
    incoming_heading_deg: float
    outgoing_heading_deg: float
    turn_angle_deg: float
    curvature_1_mm: float
    category: str
    strategy: str
    transition_start_s: float
    transition_length_mm: float
    available_length_mm: float
    required_length_mm: float | None
    feed_mm_min: float | None
    recommended_feed_mm_min: float | None
    feasible: bool | None
    required_omega_deg_s: float | None = None
    required_alpha_deg_s2: float | None = None
    orientation_error_at_vertex_deg: float = 0.0
    settling_distance_after_vertex_mm: float = 0.0
    status: str = 'PASS'

    def to_dict(self):
        return asdict(self)


class Field:
    def __init__(self, stroke: Stroke, config: Config, previous_c: float | None = None):
        self.stroke = stroke
        self.config = config
        self.s = [0.0]
        self.seg_heading = []
        self.seg_length = []
        self.corners: list[Corner] = []
        self.point_heading = []
        self.point_c = []
        self.point_curvature = []
        points = [p.xyz for p in stroke.points]
        for index, (a, b) in enumerate(zip(points, points[1:])):
            length = xy_length(a, b)
            arc = stroke.points[index+1]
            if arc.arc_center is not None:
                length = abs(math.radians(arc.arc_sweep_deg))*arc.arc_radius
            if length <= config.geometry.zero_length_tolerance_mm:
                raise ValueError('Stroke contiene segmento XY degenerado')
            self.seg_length.append(length)
            self.s.append(self.s[-1] + length)
            self.seg_heading.append(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
        if not self.seg_heading:
            return
        self.seg_unwrapped = [self.seg_heading[0]]
        for heading in self.seg_heading[1:]:
            self.seg_unwrapped.append(self.seg_unwrapped[-1] + wrap180(heading - self.seg_unwrapped[-1]))
        # Sharp and moderate vertices divide the tangent spans.
        corner_indices = set()
        n = len(points)
        closed = n > 3 and xy_length(points[0], points[-1]) <= config.geometry.zero_length_tolerance_mm
        indices = list(range(1, n - 1)) + ([n - 1] if closed else [])
        self.classifications = classify_vertices(stroke, config)
        for i in indices:
            if stroke.points[i].heading_override_deg is not None or self.classifications.get(i, {}).get('category') in ('LINE', 'SMOOTH_CURVE'):
                continue
            incoming = self.seg_unwrapped[i - 1]
            outgoing = (self.seg_unwrapped[i] if i < n - 1 else
                        nearest_equivalent_angle(self.seg_unwrapped[0], incoming, 360))
            turn = wrap180(outgoing - incoming)
            magnitude = abs(turn)
            if magnitude <= config.corner_detection.smooth_turn_max_deg:
                continue
            category = ('SHARP_CORNER' if magnitude >= config.corner_detection.sharp_corner_min_deg
                        else 'MODERATE_CORNER')
            incoming_length = self.seg_length[i - 1]
            outgoing_length = self.seg_length[i] if i < n - 1 else self.seg_length[0]
            curvature = math.radians(turn) / ((incoming_length + outgoing_length) / 2)
            available = min(config.corner_planner.max_pretransition_mm,
                            incoming_length * config.corner_planner.max_fraction_of_incoming_segment)
            feed = stroke.points[i].feed
            kin = config.kinematics
            times = []
            if kin.max_c_velocity_deg_s:
                times.append(1.875 * magnitude / kin.max_c_velocity_deg_s)
            if kin.max_c_acceleration_deg_s2:
                times.append(math.sqrt(5.7735 * magnitude / kin.max_c_acceleration_deg_s2))
            required = (feed / 60 * max(times)) if times and feed else None
            nominal = min(10.0, available)
            length = max(config.corner_planner.minimum_transition_mm, required or nominal)
            feasible = (length <= available + 1e-9) if times else None
            recommended = (60 * available / max(times)) if times and not feasible else None
            strategy = 'PRETURN_BLEND'
            status = 'PASS'
            if config.corner_planner.strategy == 'exact_vertex':
                strategy = 'EXACT_VERTEX'
                length = 0.0
                if not config.corner_planner.allow_c_only_at_sharp_corner:
                    status = 'FAIL'
            elif length > available + 1e-9:
                strategy = 'EXACT_VERTEX' if config.corner_planner.allow_c_only_at_sharp_corner else 'PRETURN_BLEND'
                length = 0.0 if strategy == 'EXACT_VERTEX' else available
                status = 'WARNING' if strategy == 'EXACT_VERTEX' else 'WARNING'
            corner = Corner(len(self.corners) + 1, stroke.id, stroke.layer_id, i,
                            stroke.points[i].line, self.s[i], incoming, outgoing, turn, curvature,
                            category, strategy, self.s[i] - length, length, available,
                            required, feed, recommended, feasible,
                            1.875 * magnitude / (length / (feed / 60)) if length and feed else None,
                            5.7735 * magnitude / (length / (feed / 60)) ** 2 if length and feed else None,
                            status=status)
            self.corners.append(corner)
            corner_indices.add(i)
        # Centered derivative in arclength, never crossing a classified corner.
        for i in range(n):
            if stroke.points[i].heading_override_deg is not None:
                h = nearest_equivalent_angle(stroke.points[i].heading_override_deg,
                                             self.seg_unwrapped[min(i, n - 2)], 360)
            elif i == 0 or i in corner_indices or (i - 1) in corner_indices:
                h = self.seg_unwrapped[min(i, n - 2)]
            elif i == n - 1:
                h = self.seg_unwrapped[-1]
            else:
                h1, h2 = self.seg_length[i - 1], self.seg_length[i]
                coeff0 = -h2 / (h1 * (h1 + h2))
                coeff1 = (h2 - h1) / (h1 * h2)
                coeff2 = h1 / (h2 * (h1 + h2))
                dx = coeff0 * points[i - 1][0] + coeff1 * points[i][0] + coeff2 * points[i + 1][0]
                dy = coeff0 * points[i - 1][1] + coeff1 * points[i][1] + coeff2 * points[i + 1][1]
                h = nearest_equivalent_angle(math.degrees(math.atan2(dy, dx)), self.seg_unwrapped[i - 1], 360)
            self.point_heading.append(h)
            if 0 < i < n - 1:
                turn = wrap180(self.seg_unwrapped[i] - self.seg_unwrapped[i - 1])
                self.point_curvature.append(math.radians(turn) / ((self.seg_length[i - 1] + self.seg_length[i]) / 2))
            else:
                self.point_curvature.append(0.0)
        # Orientations remain continuous even across multiple revolutions.
        # Quadratic one-sided derivative at free smooth endpoints avoids half-step bias.
        if n >= 3:
            for i, j, k in ((0, 1, 2), (n-1, n-2, n-3)):
                if any(stroke.points[v].heading_override_deg is not None for v in (i,j,k)) or any(v in corner_indices for v in (i,j,k)):
                    continue
                h1=abs(self.s[j]-self.s[i]); h2=abs(self.s[k]-self.s[j])
                co=(-(2*h1+h2)/(h1*(h1+h2)), (h1+h2)/(h1*h2), -h1/(h2*(h1+h2)))
                dx=sum(c*points[v][0] for c,v in zip(co,(i,j,k)))
                dy=sum(c*points[v][1] for c,v in zip(co,(i,j,k)))
                if i: dx,dy=-dx,-dy
                self.point_heading[i]=nearest_equivalent_angle(math.degrees(math.atan2(dy,dx)),self.point_heading[i],360)
        offset = config.orientation_offset_deg
        for h in self.point_heading:
            target = nozzle_angle(h, config)
            ref = self.point_c[-1] if self.point_c else previous_c
            self.point_c.append(nearest_equivalent_angle(target, ref, config.orientation.symmetry_period_deg) if ref is not None else target)
        for corner in self.corners:
            idx = corner.point_index
            ref = self.point_c[idx - 1]
            self.point_c[idx] = nearest_equivalent_angle(corner.outgoing_heading_deg + offset, ref,
                                                          config.orientation.symmetry_period_deg)
        # Analytic sweeps carry the physical revolution, including >180° and full circles.
        for i,p in enumerate(stroke.points[1:]):
            if p.arc_center is not None:
                target=self.point_c[i]+p.arc_sweep_deg
                shift=target-self.point_c[i+1]
                for j in range(i+1,len(self.point_c)): self.point_c[j]+=shift
        self.by_vertex = {corner.point_index: corner for corner in self.corners}

    def _smooth_slope(self, index: int) -> float:
        """PCHIP slope, constrained to the smooth span containing this point."""
        left = (self.point_c[index] - self.point_c[index - 1]) / self.seg_length[index - 1] if (
            index > 0 and index not in self.by_vertex and self.stroke.points[index].arc_center is None) else None
        right = (self.point_c[index + 1] - self.point_c[index]) / self.seg_length[index] if (
            index < len(self.seg_length) and index + 1 not in self.by_vertex and self.stroke.points[index+1].arc_center is None) else None
        if left is None: return right or 0.0
        if right is None: return left
        if left * right <= 0: return 0.0
        h0, h1 = self.seg_length[index - 1], self.seg_length[index]
        w0, w1 = 2 * h1 + h0, h1 + 2 * h0
        return (w0 + w1) / (w0 / left + w1 / right)

    def at(self, segment_index: int, t: float) -> float:
        """C at the exact interpolated XYZ point on one source segment."""
        start = self.point_c[segment_index]
        end = self.point_c[segment_index + 1]
        arc = self.stroke.points[segment_index+1]
        if arc.arc_center is not None:
            return start + arc.arc_sweep_deg*t
        if 'reference_orientation_pieces' in arc.fit_metadata:
            from .numerics import value
            pieces=self.polynomial_pieces(segment_index)
            for lo,hi,coeff in pieces:
                if lo-1e-12<=t<=hi+1e-12: return value(coeff,t)
            raise ValueError('Dominio de orientación de referencia incompleto')
        corner = self.by_vertex.get(segment_index + 1)
        if corner:
            if corner.strategy == 'EXACT_VERTEX':
                return start
            position = self.s[segment_index] + t * self.seg_length[segment_index]
            incoming = nearest_equivalent_angle(corner.incoming_heading_deg + self.config.orientation_offset_deg, start,
                                                self.config.orientation.symmetry_period_deg)
            if position <= corner.transition_start_s:
                fraction = (position - self.s[segment_index]) / max(1e-12, corner.transition_start_s - self.s[segment_index])
                return start + (incoming - start) * max(0.0, min(1.0, fraction))
            u = (position - corner.transition_start_s) / corner.transition_length_mm
            return incoming + (end - incoming) * quintic(u)
        h = self.seg_length[segment_index]
        m0 = self._smooth_slope(segment_index)
        m1 = self._smooth_slope(segment_index + 1)
        t2, t3 = t * t, t * t * t
        return ((2*t3 - 3*t2 + 1) * start + (t3 - 2*t2 + t) * h * m0 +
                (-2*t3 + 3*t2) * end + (t3 - t2) * h * m1)

    def xyz_at(self, i, t):
        a,b=self.stroke.points[i:i+2]
        if b.arc_center is None:
            return tuple(a.xyz[j]+(b.xyz[j]-a.xyz[j])*t for j in range(3))
        if t==1: return b.xyz
        cx,cy=b.arc_center
        angle=math.atan2(a.xyz[1]-cy,a.xyz[0]-cx)+math.radians(b.arc_sweep_deg)*t
        return (cx+b.arc_radius*math.cos(angle),cy+b.arc_radius*math.sin(angle),a.xyz[2]+(b.xyz[2]-a.xyz[2])*t)

    def polynomial_pieces(self, i):
        start,end=self.point_c[i:i+2]
        arc=self.stroke.points[i+1]
        if arc.arc_center is not None:
            return [(0.,1.,[start,arc.arc_sweep_deg])]
        if 'reference_orientation_pieces' in arc.fit_metadata:
            shift=start-arc.fit_metadata['reference_c_start']
            return [(lo,hi,[coeff[0]+shift]+list(coeff[1:]))
                    for lo,hi,coeff in arc.fit_metadata['reference_orientation_pieces']]
        h=self.seg_length[i]
        corner=self.by_vertex.get(i+1)
        if corner:
            if corner.strategy=='EXACT_VERTEX': return [(0.,1.,[start])]
            t0=max(0.,(corner.transition_start_s-self.s[i])/h)
            incoming=nearest_equivalent_angle(corner.incoming_heading_deg+self.config.orientation_offset_deg,start,self.config.orientation.symmetry_period_deg)
            result=[(0.,t0,[start,(incoming-start)/t0])] if t0>1e-12 else []
            coefficients=compose_affine([incoming,0,0,10*(end-incoming),-15*(end-incoming),6*(end-incoming)],-t0/(1-t0),1/(1-t0))
            return result+[(t0,1.,coefficients)]
        m0,m1=h*self._smooth_slope(i),h*self._smooth_slope(i+1)
        return [(0.,1.,[start,m0,3*(end-start)-2*m0-m1,2*(start-end)+m0+m1])]

    def interpolation_error(self,i,a,b,ca,cb):
        if b-a<=1e-14: return 0.0
        maximum=0.
        for lo,hi,coeff in self.polynomial_pieces(i):
            lo,hi=max(lo,a),min(hi,b)
            if hi<lo: continue
            c=list(coeff)+[0.]*(max(2,len(coeff))-len(coeff))
            slope=(cb-ca)/(b-a)
            c[0]-=ca-slope*a; c[1]-=slope
            maximum=max(maximum,max_abs(c,lo,hi)[0])
        return maximum

    def max_dc_ds(self,i):
        return max(max_abs(derivative(c),a,b)[0]/self.seg_length[i]
                   for a,b,c in self.polynomial_pieces(i))

    def theta_at(self,i,t):
        if i+1 in self.by_vertex:
            return self.seg_unwrapped[i]
        return self.at(i,t)-self.config.orientation_offset_deg

    def curvature_at(self,i,t):
        arc=self.stroke.points[i+1]
        if arc.arc_center is not None:
            return math.copysign(1/arc.arc_radius,arc.arc_sweep_deg)
        if i+1 in self.by_vertex:
            return 0.0
        from .numerics import value
        c=self.polynomial_pieces(i)[0][2]
        return math.radians(value(derivative(c),t)/self.seg_length[i])
