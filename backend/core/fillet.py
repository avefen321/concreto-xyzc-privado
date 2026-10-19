"""Local circular corner replacement. Raw strokes are never mutated."""
from dataclasses import dataclass, asdict, replace
import math
from .config import Config
from .path import Point, Stroke, xy_length
from .classification import classify_vertices


@dataclass
class Fillet:
    id: int
    stroke_id: int
    layer_id: int | None
    point_index: int
    source_line: int
    turn_angle_deg: float
    radius_mm: float
    t1: tuple[float, float, float] | None
    t2: tuple[float, float, float] | None
    center: tuple[float, float] | None
    sweep_deg: float
    tangent_distance_mm: float
    max_path_deviation_mm: float
    incoming_heading_deg: float
    outgoing_heading_deg: float
    generated_segments: int
    status: str
    reason: str | None = None
    corner_orientation_error_at_arc_exit_deg: float = 0.0
    corner_settling_distance_after_arc_mm: float = 0.0
    c_start_deg: float | None = None
    c_end_deg: float | None = None
    optimized_point_start: int | None = None
    optimized_point_end: int | None = None
    incoming_length: float = 0.0
    outgoing_length: float = 0.0
    radius_preferred_mm: float = 0.0
    radius_max_feasible_mm: float = 0.0
    radius_min_kinematic_mm: float | None = None
    required_omega_C_deg_s: float | None = None
    F_allowed_mm_min: float | None = None
    kinematic_status: str = 'NOT CONFIGURED'

    def to_dict(self):
        value = asdict(self)
        value['max_polyline_deviation_mm'] = self.max_path_deviation_mm
        value['vertex_setback_mm'] = (self.radius_mm * (1 / math.cos(math.radians(abs(self.sweep_deg))/2) - 1)
                                     if abs(self.sweep_deg) < 175 else None)
        value['corner_settling_distance_after_T2_mm'] = self.corner_settling_distance_after_arc_mm
        value['curvature_1_mm'] = math.copysign(1/self.radius_mm, self.sweep_deg)
        value['radius_selected_mm'] = self.radius_mm
        value['fillet_arc_length_mm'] = self.radius_mm*abs(math.radians(self.sweep_deg))
        value['direction'] = 'CCW' if self.sweep_deg>0 else 'CW'
        value['primitive_type'] = 'ArcPrimitive'
        return value


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point(x, y, z):
    return (float(x), float(y), float(z))


def _candidate(stroke: Stroke, index: int, config: Config, closed: bool, fillet_id: int,
               available=None) -> Fillet | None:
    points = stroke.points
    n = len(points)
    vertex = points[index].xyz
    before = points[index - 1].xyz
    after = points[1].xyz if index == n - 1 and closed else points[index + 1].xyz
    length_in, length_out = _distance(before, vertex), _distance(vertex, after)
    if length_in <= 1e-9 or length_out <= 1e-9:
        return None
    ux, uy = (vertex[0] - before[0]) / length_in, (vertex[1] - before[1]) / length_in
    vx, vy = (after[0] - vertex[0]) / length_out, (after[1] - vertex[1]) / length_out
    turn = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
    magnitude = abs(math.degrees(turn))
    if magnitude < config.corner_smoothing.detection_threshold_deg:
        return None
    radius = config.corner_smoothing.radius_mm
    base = dict(id=fillet_id, stroke_id=stroke.id, layer_id=stroke.layer_id,
                point_index=index, source_line=points[index].line,
                turn_angle_deg=math.degrees(turn), radius_mm=radius,
                t1=None, t2=None, center=None, sweep_deg=math.degrees(turn),
                tangent_distance_mm=0.0, max_path_deviation_mm=0.0,
                incoming_heading_deg=math.degrees(math.atan2(uy, ux)),
                outgoing_heading_deg=math.degrees(math.atan2(vy, vx)),
                generated_segments=0, status='FILLET_NOT_FEASIBLE')
    base.update(incoming_length=length_in,outgoing_length=length_out)
    if magnitude >= 175:
        return Fillet(**base, reason='Giro cercano a 180°')
    if max(abs(before[2] - vertex[2]), abs(after[2] - vertex[2])) > config.layers.z_change_tolerance_mm:
        return Fillet(**base, reason='Fillet reservado para esquina plana a Z constante')
    cfg=config.corner_smoothing
    free=available or (length_in-cfg.min_segment_remaining_mm,length_out-cfg.min_segment_remaining_mm)
    maximum=max(0.,min(free))/math.tan(abs(turn)/2)
    if cfg.max_path_deviation_mm is not None:
        maximum=min(maximum,cfg.max_path_deviation_mm/(1-math.cos(abs(turn)/2)))
    if cfg.max_radius_mm is not None: maximum=min(maximum,cfg.max_radius_mm)
    feed=points[index].feed
    omega=config.kinematics.max_c_velocity_deg_s
    rkin=feed/60/math.radians(omega) if omega and feed else None
    preferred=max(config.nozzle.length_mm*cfg.preferred_radius_nozzle_factor,rkin or 0.)
    if cfg.radius_mode=='adaptive': radius=min(preferred,maximum)
    base.update(radius_mm=radius,radius_preferred_mm=preferred if cfg.radius_mode=='adaptive' else cfg.radius_mm,
                radius_max_feasible_mm=maximum,radius_min_kinematic_mm=rkin)
    if radius<=config.geometry.zero_length_tolerance_mm:
        base['radius_mm']=cfg.radius_mm
        return Fillet(**base,reason='Sin longitud disponible para tangencias y guardas')
    base.update(required_omega_C_deg_s=math.degrees(feed/60/radius) if feed else None,
                F_allowed_mm_min=60*radius*math.radians(omega) if omega else None,
                kinematic_status='CORNER_KINEMATIC_LIMIT' if (cfg.radius_mode=='adaptive' and preferred>maximum+1e-9)
                or (rkin and radius<rkin-1e-9) else 'PASS' if omega else 'NOT CONFIGURED')
    d = radius * math.tan(abs(turn) / 2)
    deviation = radius * (1 - math.cos(abs(turn) / 2))
    base['tangent_distance_mm'] = d
    base['max_path_deviation_mm'] = deviation
    if d + config.corner_smoothing.min_segment_remaining_mm > min(length_in, length_out) + 1e-9:
        return Fillet(**base, reason='Radio no cabe en ambos segmentos')
    if cfg.max_path_deviation_mm is not None and deviation > cfg.max_path_deviation_mm + 1e-9:
        return Fillet(**base, reason='Desviación supera el límite configurado')
    sign = 1 if turn > 0 else -1
    t1 = _point(vertex[0] - ux * d, vertex[1] - uy * d, vertex[2])
    t2 = _point(vertex[0] + vx * d, vertex[1] + vy * d, vertex[2])
    center = (t1[0] - uy * sign * radius, t1[1] + ux * sign * radius)
    if abs(_distance(center, t1) - radius) > 1e-7 or abs(_distance(center, t2) - radius) > 1e-6:
        return Fillet(**base, reason='Geometría de tangencia inválida')
    base.update(t1=t1, t2=t2, center=center, status='PASS')
    return Fillet(**base)


def _segments_for_arc(fillet: Fillet, config: Config) -> int:
    angle = abs(math.radians(fillet.sweep_deg))
    radius = fillet.radius_mm
    chord_error = config.corner_smoothing.max_chord_error_mm
    chord_angle = 2 * math.acos(max(-1.0, min(1.0, 1 - chord_error / radius)))
    return max(2, math.ceil(abs(fillet.sweep_deg) / min(config.corner_smoothing.max_delta_c_deg, config.sampling.max_delta_c_deg)),
               math.ceil(angle / chord_angle) if chord_angle else 2,
               math.ceil(angle * radius / config.sampling.max_linear_step_mm))


def optimize_stroke(stroke: Stroke, config: Config, preserve_closed_seam: bool = False) -> tuple[Stroke, list[Fillet]]:
    """Return a separate optimized stroke and explicit accepted/rejected fillets."""
    raw = stroke.points
    if not stroke.deposition or len(raw) < 3 or config.geometry.mode != 'SMOOTH_CORNERS' or not config.corner_smoothing.enabled:
        cloned = [replace(p,original_segment=max(0,i-1)) for i,p in enumerate(raw)]
        return Stroke(stroke.id, stroke.layer_id, stroke.deposition, cloned), []
    closed = len(raw) > 3 and _distance(raw[0].xyz, raw[-1].xyz) <= config.geometry.zero_length_tolerance_mm
    indices = list(range(1, len(raw) - 1)) + ([len(raw) - 1] if closed and not preserve_closed_seam else [])
    candidates = []
    classifications = classify_vertices(stroke, config)
    corner_indices={i for i in indices if classifications.get(i,{}).get('category') not in ('LINE','SMOOTH_CURVE')
                    and abs(classifications.get(i,{}).get('turn_deg',0))>=config.corner_smoothing.detection_threshold_deg
                    and raw[i].heading_override_deg is None and not raw[i].locked}
    for index in indices:
        if index not in corner_indices:
            continue
        incoming_neighbor=index-1 if index>1 else len(raw)-1 if closed else 0
        outgoing_neighbor=index+1 if index<len(raw)-1 else 1
        before,vertex,after=raw[index-1].xyz,raw[index].xyz,raw[1 if index==len(raw)-1 else index+1].xyz
        guard=config.corner_smoothing.min_segment_remaining_mm
        available=((_distance(before,vertex)-guard)/(2 if incoming_neighbor in corner_indices else 1),
                   (_distance(vertex,after)-guard)/(2 if outgoing_neighbor in corner_indices else 1))
        candidate = _candidate(stroke, index, config, closed, len(candidates) + 1,
                               available if config.corner_smoothing.radius_mode=='adaptive' else None)
        if candidate:
            candidates.append(candidate)
    by_index = {}
    start_used = [0.0] * (len(raw) - 1)
    end_used = [0.0] * (len(raw) - 1)
    for candidate in candidates:
        if candidate.status != 'PASS':
            continue
        incoming_segment = candidate.point_index - 1
        outgoing_segment = 0 if candidate.point_index == len(raw) - 1 else candidate.point_index
        d = candidate.tangent_distance_mm
        in_length = _distance(raw[incoming_segment].xyz, raw[incoming_segment + 1].xyz)
        out_length = _distance(raw[outgoing_segment].xyz, raw[outgoing_segment + 1].xyz)
        minimum = config.corner_smoothing.min_segment_remaining_mm
        if (start_used[incoming_segment] + d + minimum > in_length + 1e-9 or
                end_used[outgoing_segment] + d + minimum > out_length + 1e-9):
            candidate.status = 'FILLET_NOT_FEASIBLE'
            candidate.reason = 'Fillets adyacentes se superponen o consumen el tramo intermedio'
            continue
        end_used[incoming_segment] = d
        start_used[outgoing_segment] = d
        candidate.generated_segments = _segments_for_arc(candidate, config) if config.sampling.mode=='legacy' else 1
        if candidate.generated_segments>config.sampling.max_samples_per_stroke:
            raise ValueError(f'Stroke {stroke.id}: max_samples_per_stroke superado por el fillet')
        by_index[candidate.point_index] = candidate
    seam = by_index.get(len(raw) - 1) if closed else None
    if seam:
        start = Point(seam.t2, raw[0].line, raw[1].motion_index, raw[1].feed,
                      0, seam.id, True, seam.outgoing_heading_deg)
    else:
        start = replace(raw[0],original_segment=0)
    optimized = [start]
    for segment in range(len(raw) - 1):
        vertex_index = segment + 1
        fillet = by_index.get(vertex_index)
        if fillet is None:
            source = raw[vertex_index]
            optimized.append(replace(source,original_segment=segment))
            continue
        source = raw[vertex_index]
        fillet.optimized_point_start = len(optimized)
        optimized.append(Point(fillet.t1, source.line, source.motion_index, source.feed,
                               segment, fillet.id, True, fillet.incoming_heading_deg))
        cx, cy = fillet.center
        angle0 = math.atan2(fillet.t1[1] - cy, fillet.t1[0] - cx)
        sweep = math.radians(fillet.sweep_deg)
        for j in range(1, fillet.generated_segments + 1):
            fraction = j / fillet.generated_segments
            angle = angle0 + sweep * fraction
            xyz = (cx + fillet.radius_mm * math.cos(angle),
                   cy + fillet.radius_mm * math.sin(angle), fillet.t1[2])
            if j == fillet.generated_segments:
                xyz = fillet.t2
            heading = fillet.incoming_heading_deg + fillet.sweep_deg * fraction
            optimized.append(Point(xyz, source.line, source.motion_index, source.feed,
                                   segment, fillet.id, True, heading, fillet.center,
                                   fillet.radius_mm, fillet.sweep_deg / fillet.generated_segments))
        fillet.optimized_point_end = len(optimized) - 1
    return Stroke(stroke.id, stroke.layer_id, stroke.deposition, optimized), candidates
