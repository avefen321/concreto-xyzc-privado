"""Stroke and layer boundaries are established before any heading calculation."""
from dataclasses import dataclass, field
import math
from .gcode import Motion, arc_geometry
from .config import Config


@dataclass
class Point:
    xyz: tuple[float, float, float]
    line: int
    motion_index: int
    feed: float | None
    original_segment: int | None = None
    fillet_id: int | None = None
    generated_by_fillet: bool = False
    heading_override_deg: float | None = None
    arc_center: tuple[float, float] | None = None
    arc_radius: float | None = None
    arc_sweep_deg: float | None = None
    source_lines: list[int] = field(default_factory=list)
    locked: bool = False
    arc_source: str | None = None
    fit_metadata: dict = field(default_factory=dict)


@dataclass
class Stroke:
    id: int
    layer_id: int | None
    deposition: bool
    points: list[Point] = field(default_factory=list)
    seam_reindexed: bool = False


@dataclass
class Layer:
    id: int
    nominal_z: float
    stroke_ids: list[int] = field(default_factory=list)


def xy_length(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def build(motions: list[Motion], config: Config) -> tuple[list[Stroke], list[Layer], list[str]]:
    strokes: list[Stroke] = []
    layers: list[Layer] = []
    warnings = []
    current: Stroke | None = None
    last_deposition_z: float | None = None
    active_layer: Layer | None = None
    for i, motion in enumerate(motions):
        distance = xy_length(motion.start, motion.end)
        is_arc=motion.kind in ('ARC_CW','ARC_CCW')
        if distance <= config.geometry.zero_length_tolerance_mm and not is_arc:
            motion.stroke_id = None
            if abs(motion.end[2] - motion.start[2]) > config.layers.z_change_tolerance_mm:
                motion.kind = 'Z_ONLY'
            current = None
            continue
        if distance < config.geometry.tiny_segment_warning_mm:
            warnings.append(f"Línea {motion.line}: segmento XY pequeño ({distance:.5f} mm)")
        deposition = motion.deposition and motion.kind != 'RAPID'
        layer_id = None
        if deposition:
            z = motion.start[2]
            if last_deposition_z is None or ((current is None or not current.deposition) and
                                             abs(z - last_deposition_z) > config.layers.z_change_tolerance_mm):
                active_layer = Layer(len(layers) + 1, z)
                layers.append(active_layer)
            layer_id = active_layer.id if active_layer else None
            last_deposition_z = motion.end[2]
        new_stroke = (current is None or current.deposition != deposition or current.layer_id != layer_id
                      or (i > 0 and motions[i-1].barrier_id != motion.barrier_id)
                      or xy_length(current.points[-1].xyz, motion.start) > config.geometry.zero_length_tolerance_mm)
        if new_stroke:
            current = Stroke(len(strokes) + 1, layer_id, deposition,
                             [Point(motion.start, motion.line, i, motion.feed)])
            strokes.append(current)
            if active_layer and deposition:
                active_layer.stroke_ids.append(current.id)
        end=Point(motion.end, motion.line, i, motion.feed,source_lines=[motion.line],
                  locked=motion.e_end is not None or 'LOCKED' in motion.block.raw.upper())
        if is_arc:
            cx,cy,r,a0,sweep=arc_geometry(motion)
            heading=math.degrees(a0)+math.copysign(90,sweep)
            current.points[-1].heading_override_deg=heading
            end.heading_override_deg=heading+math.degrees(sweep)
            end.arc_center=(cx,cy); end.arc_radius=r; end.arc_sweep_deg=math.degrees(sweep)
            end.locked=True
            end.arc_source='SOURCE_G2G3'
        current.points.append(end)
        motion.stroke_id = current.id
        motion.layer_id = layer_id
    return strokes, layers, warnings


def compress_collinear(stroke, config):
    """Exact linear cleanup before radius planning; process endpoints are locked."""
    from dataclasses import replace
    if config.sampling.mode=='legacy': return stroke
    pts=[]
    for source in stroke.points:
        p=replace(source,source_lines=list(source.source_lines))
        if len(pts)>=2:
            a,b=pts[-2:]
            u=(b.xyz[0]-a.xyz[0],b.xyz[1]-a.xyz[1])
            v=(p.xyz[0]-b.xyz[0],p.xyz[1]-b.xyz[1])
            length=math.hypot(*u)+math.hypot(*v)
            if (not b.locked and not p.locked and b.arc_center is None and p.arc_center is None and
                    b.heading_override_deg is None and b.feed==p.feed and a.xyz[2]==b.xyz[2]==p.xyz[2] and
                    u[0]*v[0]+u[1]*v[1]>0 and
                    abs(u[0]*v[1]-u[1]*v[0])<=config.precision.geometry_epsilon*length):
                p.source_lines=b.source_lines+p.source_lines
                pts.pop()
        pts.append(p)
    return Stroke(stroke.id,stroke.layer_id,stroke.deposition,pts)
