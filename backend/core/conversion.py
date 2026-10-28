"""Deterministic XYZC conversion. No orientation calculation modifies XYZ."""
from dataclasses import dataclass, asdict
from collections import defaultdict
import csv
import io
import math
from .config import Config
from .gcode import parse, interpret, linearize_arcs, arc_geometry, Motion
from .path import build, Stroke, Layer, xy_length, compress_collinear
from .layer_z import separate_layer_entries, normalize_layers
from .layer_paths import alternate_open_layer_paths
from .layer_completion import prepare_layer_closure, validate_completed_closure
from .cordon import prepare_cordon, validate_cordon
from .xy_boundary_offset import apply_xy_boundary_offset
from .initial_xy_approach import prepare_initial_xy_approach
from .sampling import CurveSampler
from .diagnostics import density, primitive_density
from .orientation import Field, nearest_equivalent_angle
from .fillet import optimize_stroke, Fillet
from .classification import classify_vertices
from .seams import plan_seam
from .simplification import simplify
from .process import plan_field, enforce_acceleration
from .states import StageTrace, FinalMotionPlan, BACKEND_VERSION, digest
from .validators import validate_final
from .arc_fitting import fit_stroke
from .primitives import build_machine_primitives, analyze_primitives, native_enabled, stroke_display_points


@dataclass
class Sample:
    sample_id: int
    layer_id: int | None
    stroke_id: int | None
    source_line: int
    motion_index: int
    original_segment: int | None
    t: float
    s_global: float
    s_stroke: float
    X: float
    Y: float
    Z: float
    heading_raw_deg: float | None
    heading_unwrapped_deg: float | None
    C_deg: float
    delta_C_deg: float
    curvature_1_mm: float
    feed_mm_min: float | None
    omega_C_deg_s: float | None
    alpha_C_deg_s2: float | None
    is_corner: bool
    corner_category: str | None
    motion_type: str
    deposition_active: bool
    E: float | None = None
    generated_by_fillet: bool = False
    fillet_id: int | None = None
    original_X: float | None = None
    original_Y: float | None = None
    original_Z: float | None = None
    emit: bool = True
    source_motion_id: int | None = None
    source_lines: list[int] | None = None
    primitive_id: str | None = None
    primitive_type: str | None = None
    tangent_xy: list[float] | None = None
    nozzle_vector_xy: list[float] | None = None
    c_target_deg: float | None = None
    feed_source: float | None = None
    feed_recommended: float | None = None
    generated_reason: str = 'ORIGINAL'
    geometry_error_mm: float = 0.0
    orientation_error_deg: float = 0.0
    field_segment: int = 0
    anchor_line: int | None = None
    duration_s: float | None = None
    z_source: float | None = None
    z_target: float | None = None
    layer_index: int | None = None
    z_conversion_applied: bool = False
    z_conversion_reason: str = 'DISABLED'
    z_role: str = 'TRAVEL_Z'
    c_reference_shift_deg: float = 0.0
    segment_length_mm: float = 0.0
    feed_allowed_by_c_mm_min: float | None = None
    layer_closure_role: str | None = None
    closure_reference_line: int | None = None
    cordon_role: str | None = None
    display_only: bool = False
    curve_certified: bool = False
    c_motion_start_deg: float | None = None

    def to_dict(self):
        return asdict(self)


@dataclass
class Result:
    samples: list[Sample]
    gcode: str
    report: dict
    csv_text: str
    simulation: dict
    final_motion_plan: FinalMotionPlan | None = None


def _point_on_segment(a, b, t):
    return tuple(a[j] + (b[j] - a[j]) * t for j in range(3))


def _closest_on_segment(point, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    length2 = vx * vx + vy * vy
    if length2 <= 1e-20:
        return a
    t = max(0.0, min(1.0, ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) / length2))
    return _point_on_segment(a, b, t)


def _original_projection(point, raw_stroke: Stroke, segment_index: int, fillet: Fillet | None):
    indices = [segment_index]
    if fillet is not None:
        outgoing = 0 if fillet.point_index == len(raw_stroke.points) - 1 else fillet.point_index
        indices = [fillet.point_index - 1, outgoing]
    candidates = []
    for index in indices:
        if 0 <= index < len(raw_stroke.points) - 1:
            a, b = raw_stroke.points[index].xyz, raw_stroke.points[index + 1].xyz
            candidates.append(_closest_on_segment(point, a, b))
    return min(candidates, key=lambda p: xy_length(point, p)) if candidates else point


def _fixed_decimal(value: float, decimals: int) -> str:
    formatted = f'{value:.{decimals}f}'
    # Avoid negative zero in machine output without changing the selected precision.
    return f'{0:.{decimals}f}' if float(formatted) == 0 else formatted


def _format_coordinate(value: float, config: Config) -> str:
    return _fixed_decimal(value, config.serialization.xyz_decimals)


def _sample_segment(field: Field, i: int, config: Config) -> list[float]:
    """Return endpoint fractions after recursive angular and geometric checks."""
    result = []
    max_depth = 20
    def visit(a, b, depth):
        ca, cb = field.at(i, a), field.at(i, b)
        mid = (a + b) / 2
        length = math.dist(field.xyz_at(i,a),field.xyz_at(i,b))
        error = field.interpolation_error(i,a,b,ca,cb)
        arc = field.stroke.points[i+1]
        chord_error = arc.arc_radius*(1-math.cos(math.radians(arc.arc_sweep_deg*(b-a))/2)) if arc.arc_center else 0.
        needs_split = (length > config.sampling.max_linear_step_mm + config.precision.geometry_epsilon or
             abs(cb - ca) > config.sampling.max_delta_c_deg + config.precision.angle_epsilon_deg or
             error > config.sampling.max_orientation_interp_error_deg + config.precision.angle_epsilon_deg or
             chord_error > config.corner_smoothing.max_chord_error_mm + config.precision.geometry_epsilon)
        if needs_split and depth>=max_depth:
            raise ValueError('No se puede cumplir la tolerancia de sampling con la profundidad máxima')
        if needs_split:
            visit(a, mid, depth + 1)
            visit(mid, b, depth + 1)
        else:
            result.append(b)
    visit(0.0, 1.0, 0)
    return result


def _validate_sample_geometry(samples: list[Sample], strokes: list[Stroke], config: Config):
    by_id = {s.id: s for s in strokes}
    errors = []
    max_error = 0.0
    per_layer = defaultdict(list)
    for sample in samples:
        if not all(math.isfinite(v) for v in (sample.X, sample.Y, sample.Z, sample.C_deg)):
            errors.append(f"Línea {sample.source_line}: coordenada no finita")
            continue
        if sample.stroke_id is None or sample.original_segment is None:
            continue
        stroke = by_id[sample.stroke_id]
        if sample.original_X is not None:
            intended = (sample.original_X, sample.original_Y, sample.original_Z)
        else:
            a = stroke.points[sample.original_segment].xyz
            b = stroke.points[sample.original_segment + 1].xyz
            intended = _point_on_segment(a, b, sample.t)
        distance = xy_length((sample.X, sample.Y, 0), (intended[0], intended[1], 0))
        max_error = max(max_error, distance)
        if stroke.deposition:
            per_layer[sample.layer_id].append(distance)
        limit = (config.corner_smoothing.max_path_deviation_mm
                 if sample.generated_by_fillet and config.geometry.mode == 'SMOOTH_CORNERS'
                 else min(config.arc_fit.geometry_tolerance_mm, config.geometry.max_xy_deviation_mm)
                 if sample.generated_reason=='FREE_CURVE_FIT' and config.geometry.mode=='PRESERVE_PATH'
                 else config.arc_fit.geometry_tolerance_mm if sample.generated_reason=='FREE_CURVE_FIT'
                 else config.geometry.max_xy_deviation_mm)
        if distance > (limit if limit is not None else float('inf')) + 1e-9:
            errors.append(f"FAIL_GEOMETRY_DRIFT capa {sample.layer_id}, línea {sample.source_line}: "
                          f"original={intended[:2]}, convertido={(sample.X, sample.Y)}, error={distance:.9f} mm")
        rounded = (float(_format_coordinate(sample.X, config)), float(_format_coordinate(sample.Y, config)))
        exported_distance = xy_length(rounded, intended)
        rounding_budget = math.sqrt(2)*0.5*10**(-config.serialization.xyz_decimals)
        if exported_distance > (limit if limit is not None else float('inf')) + rounding_budget + 1e-9:
            errors.append(f"FAIL_EXPORTED_GEOMETRY_DRIFT capa {sample.layer_id}, línea {sample.source_line}: "
                          f"error={exported_distance:.9f} mm; aumente decimales XYZ")
    return max_error, per_layer, errors


def _layer_report(layers: list[Layer], strokes: list[Stroke], samples: list[Sample], per_layer, config):
    tolerance = (config.corner_smoothing.max_path_deviation_mm if config.geometry.mode == 'SMOOTH_CORNERS'
                 and config.corner_smoothing.enabled else config.geometry.max_xy_deviation_mm)
    if tolerance is None: tolerance=float('inf')
    stroke_by_id = {s.id: s for s in strokes}
    converted_by_stroke = defaultdict(list)
    converted_by_layer = defaultdict(list)
    for sample in samples:
        if sample.stroke_id is not None and sample.deposition_active:
            point=(sample.X,sample.Y,sample.Z)
            converted_by_stroke[sample.stroke_id].append(point)
            converted_by_layer[sample.layer_id].append(point)
    output = []
    for layer in layers:
        original = [p.xyz for sid in layer.stroke_ids for p in stroke_by_id[sid].points]
        converted = converted_by_layer[layer.id]
        if not original or not converted:
            continue
        def bbox(points):
            return [min(p[0] for p in points), min(p[1] for p in points),
                    max(p[0] for p in points), max(p[1] for p in points)]
        # Centroid is length weighted; densification therefore cannot shift it.
        # Each stroke separately, so centroid does not count inter-stroke travel.
        def layer_centroid(paths):
            sx = sy = total = 0.0
            for points in paths:
                for a, b in zip(points, points[1:]):
                    length = xy_length(a, b)
                    sx += (a[0] + b[0]) / 2 * length
                    sy += (a[1] + b[1]) / 2 * length
                    total += length
            return [sx / total, sy / total] if total else [0, 0]
        original_paths = [[p.xyz for p in stroke_by_id[sid].points] for sid in layer.stroke_ids]
        converted_paths = [converted_by_stroke[sid] for sid in layer.stroke_ids]
        c0 = layer_centroid(original_paths)
        c1 = layer_centroid(converted_paths)
        analytic_length=0.; cx_sum=cy_sum=0.; has_arc=False
        for sid in layer.stroke_ids:
            for a,b in zip(stroke_by_id[sid].points,stroke_by_id[sid].points[1:]):
                if b.arc_center is None:
                    length=xy_length(a.xyz,b.xyz); mx=(a.xyz[0]+b.xyz[0])/2; my=(a.xyz[1]+b.xyz[1])/2
                else:
                    has_arc=True; ox,oy=b.arc_center; r=b.arc_radius
                    phi=math.atan2(a.xyz[1]-oy,a.xyz[0]-ox); sweep=math.radians(b.arc_sweep_deg)
                    length=abs(sweep)*r
                    mx=ox+r*(math.sin(phi+sweep)-math.sin(phi))/sweep
                    my=oy+r*(math.cos(phi)-math.cos(phi+sweep))/sweep
                    for theta in (0,math.pi/2,math.pi,3*math.pi/2):
                        distance=((theta-phi) if sweep>0 else (phi-theta))%(2*math.pi)
                        if distance<=abs(sweep)+1e-12: original.append((ox+r*math.cos(theta),oy+r*math.sin(theta),a.xyz[2]))
                analytic_length+=length; cx_sum+=mx*length; cy_sum+=my*length
        if has_arc and analytic_length: c0=[cx_sum/analytic_length,cy_sum/analytic_length]
        layer_tolerance=max(tolerance,config.corner_smoothing.max_chord_error_mm) if has_arc else tolerance
        deviations = per_layer.get(layer.id, [])
        drift = math.dist(c0, c1)
        output.append({"layer_id": layer.id, "nominal_z": layer.nominal_z,
                       "start_xy": list(original_paths[0][0][:2]), "end_xy": list(original_paths[-1][-1][:2]),
                       "original_bbox_xy": bbox(original), "converted_bbox_xy": bbox(converted),
                       "original_centroid_xy": c0, "converted_centroid_xy": c1,
                       "path_length_mm": analytic_length,
                       "original_points": len(original), "converted_points": len(converted),
                       "max_xy_deviation_mm": max(deviations, default=0.0),
                       "rms_xy_deviation_mm": math.sqrt(sum(v * v for v in deviations) / len(deviations)) if deviations else 0.0,
                       "layer_xy_drift_mm": drift,
                       "status": "PASS" if max(deviations, default=0) <= layer_tolerance and drift <= layer_tolerance else "FAIL"})
    return output


def _interlayer_consistency(layers: list[Layer], raw_strokes: list[Stroke], optimized_strokes: list[Stroke]):
    raw_by_id = {s.id: s for s in raw_strokes}
    optimized_by_id = {s.id: s for s in optimized_strokes}
    groups = {}
    max_error = 0.0
    comparisons = []
    def canonical(stroke):
        points=[(p.xyz[0],p.xyz[1]) for p in stroke.points]
        closed=len(points)>3 and math.dist(points[0],points[-1])<1e-7
        if not closed: return tuple(points)
        points=points[:-1]
        # Remove only redundant collinear seam splits for comparison, never for execution.
        reduced=[]
        for i,p in enumerate(points):
            a,b=points[i-1],points[(i+1)%len(points)]
            cross=(p[0]-a[0])*(b[1]-p[1])-(p[1]-a[1])*(b[0]-p[0])
            dot=(p[0]-a[0])*(b[0]-p[0])+(p[1]-a[1])*(b[1]-p[1])
            if abs(cross)<=1e-9*max(1.,math.dist(a,p)+math.dist(p,b)) and dot>=0: continue
            reduced.append(p)
        if not reduced: return tuple(points)
        first=min(range(len(reduced)),key=lambda i:(round(reduced[i][0],7),round(reduced[i][1],7)))
        return tuple(reduced[first:]+reduced[:first])
    for layer in layers:
        signature = tuple(tuple((round(p[0],7),round(p[1],7)) for p in canonical(raw_by_id[sid])) for sid in layer.stroke_ids)
        optimized_xy = tuple(canonical(optimized_by_id[sid]) for sid in layer.stroke_ids)
        if signature in groups:
            reference_layer, reference = groups[signature]
            if len(reference) != len(optimized_xy) or any(len(a) != len(b) for a, b in zip(reference, optimized_xy)):
                left=[p for path in reference for p in path]
                right=[p for path in optimized_xy for p in path]
                error=max([min(math.dist(p,q) for q in right) for p in left]+
                          [min(math.dist(p,q) for q in left) for p in right])
            else:
                error = max((math.dist(a, b) for path_a, path_b in zip(reference, optimized_xy)
                             for a, b in zip(path_a, path_b)), default=0.0)
            max_error = max(max_error, error)
            comparisons.append({"reference_layer": reference_layer, "layer_id": layer.id,
                                "max_optimized_xy_difference_mm": error})
        else:
            groups[signature] = (layer.id, optimized_xy)
    return max_error, comparisons


def _kinematics(samples: list[Sample], config: Config):
    max_omega = max_alpha = 0.0
    prev_by_stroke = {}
    previous_omega = {}
    warnings = []
    known = 0
    unknown = 0
    for sample in samples:
        sample.omega_C_deg_s = sample.alpha_C_deg_s2 = sample.duration_s = None
        if (sample.generated_reason == 'INITIAL_C_ALIGNMENT' and
                sample.c_motion_start_deg is not None):
            sample.delta_C_deg = sample.C_deg - sample.c_motion_start_deg
            if sample.feed_mm_min and abs(sample.delta_C_deg) > 1e-12:
                sample.duration_s = 60 * abs(sample.delta_C_deg) / sample.feed_mm_min
                sample.omega_C_deg_s = sample.delta_C_deg / sample.duration_s
                max_omega = max(max_omega, abs(sample.omega_C_deg_s))
                known += 1
            continue
        if sample.stroke_id is None:
            continue
        prior = prev_by_stroke.get(sample.stroke_id)
        if prior:
            ds = math.dist((sample.X, sample.Y, sample.Z), (prior.X, prior.Y, prior.Z))
            if sample.primitive_type=='ARC' and sample.curvature_1_mm:
                ds=math.hypot(math.radians(sample.C_deg-prior.C_deg)/sample.curvature_1_mm,sample.Z-prior.Z)
            dt = ds / (sample.feed_mm_min / 60) if sample.feed_mm_min and ds else 0
            sample.delta_C_deg = sample.C_deg - prior.C_deg
            if config.kinematics.max_c_velocity_deg_s and abs(sample.delta_C_deg)>1e-12:
                c_time=abs(sample.delta_C_deg)/config.kinematics.max_c_velocity_deg_s
                sample.feed_allowed_by_c_mm_min=60*ds/max(dt,c_time) if ds else None
            if dt:
                known += 1
                sample.duration_s = dt
                sample.omega_C_deg_s = sample.delta_C_deg / dt
                max_omega = max(max_omega, abs(sample.omega_C_deg_s))
                if sample.stroke_id in previous_omega:
                    prev_w, prev_t = previous_omega[sample.stroke_id]
                    sample.alpha_C_deg_s2 = (sample.omega_C_deg_s - prev_w) / ((dt + prev_t) / 2)
                    max_alpha = max(max_alpha, abs(sample.alpha_C_deg_s2))
                previous_omega[sample.stroke_id] = sample.omega_C_deg_s, dt
            elif sample.emit:
                unknown += 1
        prev_by_stroke[sample.stroke_id] = sample
    kin = config.kinematics
    status = 'NOT CONFIGURED' if kin.max_c_velocity_deg_s is None and kin.max_c_acceleration_deg_s2 is None else 'PASS'
    if (not known or unknown) and status=='PASS': status='WARNING'
    if unknown:
        warnings.append(f'{unknown} movimientos sin duración calculable (F desconocido o giro C-only)')
    if kin.max_c_velocity_deg_s and max_omega > kin.max_c_velocity_deg_s + 1e-6:
        status = 'WARNING'; warnings.append(f"Demanda C {max_omega:.2f} °/s supera límite configurado")
    if kin.max_c_acceleration_deg_s2 and max_alpha > kin.max_c_acceleration_deg_s2 + 1e-6:
        status = 'WARNING'; warnings.append(f"Demanda C {max_alpha:.2f} °/s² supera límite configurado")
    return {"status": status, "max_omega_c_deg_s": max_omega if known else None,
            "max_alpha_c_deg_s2": max_alpha if known else None,
            "unknown_duration_segments": unknown,
            "configured_max_velocity_deg_s": kin.max_c_velocity_deg_s,
            "configured_max_acceleration_deg_s2": kin.max_c_acceleration_deg_s2}, warnings


def _serialize(blocks, motions: list[Motion], plan: FinalMotionPlan, config: Config):
    samples=plan.primitives
    conversion_id=plan.conversion_id
    geometry_only = config.serialization.motion_only
    by_line = defaultdict(list)
    for sample in samples:
        by_line[sample.anchor_line or sample.source_line].append(sample)
    lines=[]; mapping=[]
    def add(text, source_line=None, sample=None, reason='PRESERVED_COMMAND'):
        lines.append(text)
        mapping.append({'output_line':len(lines),'source_line':sample.source_line if sample else source_line,
                        'sample_id':sample.sample_id if sample else None,
                        'primitive_id':sample.id if sample else None,
                        'reason':sample.generated_reason if sample else reason})
    add('(Concrete CNC XYZC '+BACKEND_VERSION+'; conversion_id='+conversion_id+')',reason='NORMALIZED_HEADER')
    add('G21',reason='NORMALIZED_UNITS'); add('G90',reason='NORMALIZED_COORDINATES')
    add('G17',reason='NORMALIZED_PLANE'); add('G91.1',reason='NORMALIZED_ARC_CENTERS'); add('G94',reason='NORMALIZED_FEED_MODE')
    initial_xy = next((s for s in samples if s.generated_reason=='INITIAL_XY_APPROACH'),None)
    if initial_xy is not None:
        add(f'(INICIO SOLO XY: situar la maquina previamente en Z={initial_xy.Z:.3f} mm; no se ordena subida ni bajada Z inicial)',
            reason='INITIAL_XY_Z_PRECONDITION')
    if config.linuxcnc.path_mode=='G64':
        add('G64'+(f' P{config.linuxcnc.blend_tolerance_mm:g}' if config.linuxcnc.blend_tolerance_mm is not None else ''),reason='CONFIGURED_PATH_MODE')
    if not geometry_only and any(block.value('E') is not None for block in blocks): add('M82',reason='NORMALIZED_EXTRUSION')
    def coordinate(value):
        return _format_coordinate(value,config)
    def angle(value):
        return _fixed_decimal(value,config.serialization.c_decimals)
    def feed(value):
        out=_fixed_decimal(value,config.serialization.feed_decimals)
        if float(out)<=0: raise ValueError('F redondeada a cero; aumente decimales F en Salida')
        return out
    def emit(sample, restore_path_mode='G64'):
        initial_alignment = sample.generated_reason == 'INITIAL_C_ALIGNMENT'
        if initial_alignment:
            add('G61.1', source_line=sample.source_line, reason='INITIAL_C_EXACT_STOP')
        command='G0' if sample.motion_type in ('RAPID','PREP_RAPID') else 'G1'
        if sample.generated_reason=='INITIAL_XY_APPROACH':
            text=f'G0 X{coordinate(sample.X)} Y{coordinate(sample.Y)} C{angle(sample.C_deg)}'
        elif sample.motion_type=='C_ONLY':
            text=f'G1 C{angle(sample.C_deg)}'
            if initial_alignment and sample.feed_mm_min is not None:
                text+=' F'+feed(sample.feed_mm_min)
            elif config.kinematics.max_c_velocity_deg_s:
                text+=' F'+feed(60*config.kinematics.max_c_velocity_deg_s)
            elif sample.feed_mm_min is not None:
                text+=' F'+feed(sample.feed_mm_min)
        elif sample.type=='ARC':
            from .arc_serialization import arc_words
            command='G3' if sample.direction=='CCW' else 'G2'
            full_circle=(abs(abs(math.degrees(sample.sweep_rad))-360.)<=1e-7 and
                         math.dist(sample.start_xyz[:2],sample.end_xyz[:2])<=1e-7)
            if full_circle:
                # LinuxCNC center format represents a complete circle by
                # omitting the in-plane endpoint.  Z is present only for a
                # helical circle; C remains synchronized with the XY motion.
                axes=(f' Z{coordinate(sample.Z)}'
                      if abs(sample.Z-sample.start_xyz[2])>1e-9 else '')
            else:
                axes=f' X{coordinate(sample.X)} Y{coordinate(sample.Y)} Z{coordinate(sample.Z)}'
            text=f'{command}{axes} '+arc_words(sample,config)+f' C{angle(sample.C_deg)}'
            if sample.E is not None and not geometry_only: text+=' E'+coordinate(sample.E)
            if sample.feed_mm_min is not None: text+=' F'+feed(sample.feed_mm_min)
        else:
            text=f'{command} X{coordinate(sample.X)} Y{coordinate(sample.Y)} Z{coordinate(sample.Z)} C{angle(sample.C_deg)}'
            if sample.E is not None and not geometry_only: text+=' E'+coordinate(sample.E)
            if sample.feed_mm_min is not None and command=='G1': text+=' F'+feed(sample.feed_mm_min)
        add(text,sample=sample)
        if initial_alignment:
            add(restore_path_mode, source_line=sample.source_line, reason='INITIAL_C_RESTORE_PATH_MODE')
    unit=1.; e_absolute=True; e_position=0.; e_offset=0.
    configured_path_mode = ('G64'+(f' P{config.linuxcnc.blend_tolerance_mm:g}'
                            if config.linuxcnc.blend_tolerance_mm is not None else '')
                            if config.linuxcnc.path_mode=='G64' else None)
    active_path_mode = configured_path_mode or 'G64'
    for block in blocks:
        g=[v for k,v in block.words if k=='G']
        m=[v for k,v in block.words if k=='M']
        for code in g:
            if code in (20,21): unit=25.4 if code==20 else 1.
        for code in m:
            if code in (82,83): e_absolute=code==82
        e=block.value('E')
        if e is not None:
            if 92 in g: e_offset=e_position-e*unit
            else: e_position=e*unit+e_offset if e_absolute else e_position+e*unit
        members=by_line.get(block.line,[])
        source_path_codes=[code for code in g if code in (61,61.1,64)]
        block_path_mode=None
        if source_path_codes:
            selected=source_path_codes[-1]
            block_path_mode=f'G{selected:g}'
            if selected==64:
                p,q=block.value('P'),block.value('Q')
                if p is not None: block_path_mode+=f' P{p:g}'
                if q is not None: block_path_mode+=f' Q{q:g}'
        restore_path_mode=configured_path_mode or block_path_mode or active_path_mode
        prep_end=max((i+1 for i,s in enumerate(members)
                      if s.motion_type=='PREP_RAPID' or s.generated_reason=='INITIAL_C_ALIGNMENT'),
                     default=0) if any(code in (3,4) for code in m) else 0
        for sample in members[:prep_end]: emit(sample,restore_path_mode)
        if 92 in g: add(f'(source G92 line {block.line}: resolved absolute coordinates)',block.line)
        passthrough=[]; terminal=[]
        for k,v in block.words:
            if geometry_only:
                if k=='M':
                    if v in (2,30): terminal.append(f'M{v:g}')
                    continue
                if k not in ('G','F') and not (k in ('P','Q') and any(gv in (61,61.1,64) for gv in g)):
                    continue
                if k=='G' and v not in (0,1,2,3,17,18,19,20,21,90,91,92,61,61.1,64):
                    continue
            if config.linuxcnc.path_mode=='G64' and (k=='G' and v in (61,61.1,64) or k in ('P','Q') and any(gv in (61,61.1,64) for gv in g)):
                continue
            if k=='G' and v in (0,1,2,3,17,18,19,20,21,90,91,90.1,91.1,92,94): continue
            if k=='P' and any(gv in (2,3) for gv in g): continue
            if k=='M' and v in (82,83): continue
            if k=='M' and v in (2,30): terminal.append(f'M{v:g}'); continue
            if k in 'XYZCIJKREF': continue
            if k=='G' and not config.serialization.preserve_unknown_commands and v not in (61,61.1,64): continue
            passthrough.append(f'{k}{v:g}')
        if passthrough: add(' '.join(passthrough),block.line)
        if config.serialization.preserve_comments:
            for comment in block.comments: add(comment,block.line,reason='COMMENT')
        for sample in members[prep_end:]: emit(sample,restore_path_mode)
        if configured_path_mode is None and block_path_mode is not None:
            active_path_mode=block_path_mode
        if (not members and block.value('F') is not None and not any(block.value(k) is not None for k in 'XYZ')
                and not any(gv in (2,3) for gv in g)):
            add('F'+feed(block.value('F')*unit),block.line,reason='NORMALIZED_FEED')
        if not geometry_only and e is not None and not members and 92 not in g and not any(block.value(k) is not None for k in 'XYZ'):
            add('G1 E'+coordinate(e_position),block.line,reason='NORMALIZED_EXTRUSION')
        for command in terminal: add(command,block.line)
        if terminal: break
    return '\n'.join(lines)+'\n',mapping


def _csv(samples):
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=list(Sample.__dataclass_fields__), lineterminator='\n')
    writer.writeheader()
    for sample in samples:
        writer.writerow(sample.to_dict())
    return output.getvalue()


def convert(source: str, config: Config | None = None, *, representation=None) -> Result:
    config = config or Config()
    config.validate()
    trace_stages = StageTrace()
    input_hash, config_hash = digest(source), digest(config.to_dict())
    conversion_id = digest([input_hash,config_hash,BACKEND_VERSION])
    if representation is not None:
        representation.verify(source,config)
        conversion_id=digest([conversion_id,representation.identity])
    trace_stages.add(source,len(source.splitlines()))
    blocks = parse(source)
    parsed_program=[asdict(b) for b in blocks]
    trace_stages.add(parsed_program,len(blocks))
    motions, warnings = interpret(blocks)
    path_modes=[{'source_line':b.line,'mode':f'G{v:g}','P':b.value('P'),'Q':b.value('Q')}
                for b in blocks for k,v in b.words if k=='G' and v in (61,61.1,64)]
    if config.linuxcnc.path_mode=='PRESERVE_SOURCE' and any(r['mode'] in ('G61','G61.1') for r in path_modes):
        warnings.append('EXACT_PATH_MODE: entrada contiene G61/G61.1; puede causar desaceleraciones o paradas')
    input_motion_count = len(motions)
    input_counts={name:sum(m.kind==kind for m in motions) for name,kind in
                  (('input_G1','LINEAR'),('input_G2','ARC_CW'),('input_G3','ARC_CCW'))}
    if any(m.deposition and m.block.value('C') is not None for m in motions):
        warnings.append('SOURCE_C_RECOMPUTED: C de deposición se recalcula desde la geometría final y la alineación configurada; valores de origen registrados en el plan canónico')
    motions, closure_report, closure_warnings = prepare_layer_closure(motions, blocks, config)
    warnings.extend(closure_warnings)
    motions, cordon_report = prepare_cordon(motions, blocks, config, closure_report)
    motions, initial_approach_report = prepare_initial_xy_approach(motions, config)
    motions, xy_offset_report = apply_xy_boundary_offset(motions, cordon_report, config)
    if config.sampling.mode=='legacy':
        motions = linearize_arcs(motions,min(1.,config.sampling.max_linear_step_mm),
                                 min(2.,config.sampling.max_delta_c_deg),min(.03,config.corner_smoothing.max_chord_error_mm))
    motions,entry_warnings=separate_layer_entries(motions,config)
    warnings.extend(entry_warnings)
    strokes, layers, path_warnings = build(motions, config)
    design_strokes=[{'stroke_id':s.id,'layer_id':s.layer_id,'deposition':s.deposition,
                     'cordon': bool(s.points[1:] and all(motions[p.motion_index].cordon_role == 'CORDON_PRIME'
                                                   for p in s.points[1:])),
                     'points':stroke_display_points(s,config)} for s in strokes]
    design_path = tuple((s.id,s.layer_id,s.deposition,tuple(p.xyz for p in s.points)) for s in strokes)
    strokes,layers,open_transition_report,transition_warnings=alternate_open_layer_paths(
        strokes,layers,motions,config)
    warnings.extend(transition_warnings)
    z_report=normalize_layers(strokes,layers,motions,config)
    if initial_approach_report['status']=='XY_ONLY_REQUIRES_PREPOSITIONED_Z' and layers:
        initial_approach_report['required_start_z_mm']=layers[0].nominal_z
        warnings.append(f"INITIAL_XY_APPROACH: ubique físicamente Z={layers[0].nominal_z:g} mm antes de ejecutar; la aproximación inicial solo mueve XY.")
    strokes=[compress_collinear(s,config) for s in strokes]
    canonical_program=[{'canonical_motion_id':i,'source_motion_id':m.source_motion_id,'source_line':m.line,
                        'start_xyz':list(m.start),'end_xyz':list(m.end),'kind':m.kind,'feed_source':m.feed,
                        'deposition':m.deposition,'layer_id':m.layer_id,'stroke_id':m.stroke_id,
                        'layer_closure_role':m.layer_closure_role,'closure_reference_line':m.closure_reference_line,
                        'cordon_role':m.cordon_role,
                        'arc':m.arc,'source_c_start':m.c_start,'source_c_end':m.c_end,
                        'barrier_id':m.barrier_id} for i,m in enumerate(motions)]
    trace_stages.add(canonical_program,len(motions))
    classifications = [{'stroke_id':s.id,'layer_id':s.layer_id,
                        'vertices':[{**v,'vertex_xyz':list(s.points[v['point_index']].xyz)} for v in classify_vertices(s,config).values()]} for s in strokes]
    trace_stages.add(classifications,sum(len(s['vertices']) for s in classifications))
    warnings.extend(path_warnings)
    if config.geometry.mode == 'SMOOTH_CORNERS' and config.corner_smoothing.enabled and any(
            motion.e_end is not None and motion.deposition for motion in motions):
        raise ValueError('SMOOTH_CORNERS con eje E no está implementado; seleccione PRESERVE_PATH')
    fields = {}
    optimized_strokes = []
    all_fillets: list[Fillet] = []
    fillet_by_id = {}
    samples: list[Sample] = []
    s_global = 0.0
    initial_orientation_mode = config.orientation.initial_orientation_mode
    fixed_axis_start = initial_orientation_mode == 'ROTATE_AT_START'
    previous_c = 0.0 if fixed_axis_start else None
    c_reference_shift = -config.orientation.home_axis_deg if fixed_axis_start else 0.0
    initial_alignment_sample = None
    initial_alignment_start_c = None
    initial_alignment_target_c = None
    initial_alignment_xyz = None
    initial_alignment_activation_line = None
    initial_alignment_unsafe_activation = False
    safe_seams=[]
    feed_rows=[]
    execution_order={}
    curve_spans=[]
    sampler_input=0
    arc_fit_decisions=[]
    reference_point_counts={}
    last_print_motion_index=0
    seam_config=config
    if cordon_report['status'] in ('ADDED','ADJUSTED','KEPT') and config.layer_transition.safe_seam:
        seam_config=Config.from_dict(config.to_dict())
        seam_config.layer_transition.safe_seam=False
        warnings.append('SAFE_SEAM_DISABLED_FOR_CORDON: el cordón fija el inicio de la figura en todas las capas.')
    cordon_layer_id = next((m.layer_id for m in motions if m.cordon_role == 'CORDON_PRIME'), None)
    for stroke in strokes:
        cordon_end = cordon_report.get('actual_end_xyz')
        preserve_cordon_seam = (cordon_end is not None and stroke.deposition and
                                stroke.layer_id == cordon_layer_id and
                                len(stroke.points) > 1 and
                                all(abs(a-b) <= config.geometry.zero_length_tolerance_mm
                                    for a,b in zip(stroke.points[0].xyz[:2], cordon_end[:2])) and
                                not all(motions[p.motion_index].cordon_role == 'CORDON_PRIME'
                                        for p in stroke.points[1:]))
        optimized, fillets = optimize_stroke(stroke, config, preserve_cordon_seam)
        optimized, seam_report = plan_seam(optimized,fillets,seam_config)
        reference_point_counts[stroke.id]=len(optimized.points)
        # Curve reconstruction depends on relative orientation change, not on
        # the machine's startup reference.  Preserve the established first-
        # stroke reference for fitting; apply the fixed-axis C0 only when the
        # final machine field is built below.
        fit_previous_c=(None if fixed_axis_start and not fields else previous_c)
        if config.arc_fit.reference_model=='PARAMETRIC_HERMITE':
            # Reconstruction is geometric. A machine's accumulated C branch
            # must not change its partitions/fallbacks on repeated layers.
            # The final Field below still unwraps C against previous_c.
            fit_previous_c=None
        optimized,fit_decisions=fit_stroke(optimized,config,fit_previous_c,representation)
        arc_fit_decisions.extend(fit_decisions)
        if stroke.deposition:
            safe_seams.append(seam_report)
            if seam_report['status']=='WARNING': warnings.append(f"Trazo {stroke.id}: {seam_report['reason']}")
        optimized_strokes.append(optimized)
        all_fillets.extend(fillets)
        for fillet in fillets:
            fillet_by_id[(stroke.id, fillet.id)] = fillet
            if fillet.kinematic_status=='CORNER_KINEMATIC_LIMIT':
                warnings.append(f'Línea {fillet.source_line}: CORNER_KINEMATIC_LIMIT: radio {fillet.radius_mm:.3f} mm limitado por geometría; F permitida {fillet.F_allowed_mm_min}')
            if fillet.status != 'PASS':
                warnings.append(f"Línea {fillet.source_line}: FILLET_NOT_FEASIBLE: {fillet.reason}")
        if not stroke.deposition:
            continue
        for preceding in motions[last_print_motion_index:int(stroke.points[0].motion_index)]:
            if preceding.block.value('C') is not None: previous_c=preceding.c_end
        incoming_c=previous_c
        first_print_stroke=not fields
        field = Field(optimized, config, previous_c,
                      home_reference=(first_print_stroke and previous_c is None and
                                      initial_orientation_mode == 'HOME_ALIGNED_TANGENT'),
                      reference_shift_deg=c_reference_shift)
        if representation is not None:
            representation.align_field(stroke.id,field)
        c_reference_shift=field.reference_shift_deg
        planned_feeds, recommendations = plan_field(field,config)
        feed_rows.extend(recommendations)
        fields[stroke.id] = field
        if field.point_c:
            previous_c = (representation.end_c(stroke.id,field.point_c[-1])
                          if representation is not None else field.point_c[-1])
        last_print_motion_index=int(stroke.points[-1].motion_index)+1
        if len(field.corners) and any(c.status == 'FAIL' for c in field.corners):
            warnings.append(f"Stroke {stroke.id}: EXACT_VERTEX requiere permitir C-only")
        if any(c.feasible is False for c in field.corners):
            warnings.append(f"Stroke {stroke.id}: CORNER_NOT_KINEMATICALLY_FEASIBLE")
        start = optimized.points[0]
        stroke_sample_start=len(samples)
        start_fillet = fillet_by_id.get((stroke.id, start.fillet_id)) if start.fillet_id else None
        original_start = _original_projection(start.xyz, stroke, start.original_segment or 0, start_fillet)
        samples.append(Sample(len(samples) + 1, stroke.layer_id, stroke.id, start.line, start.motion_index,
                              start.original_segment or 0, 0, s_global, 0, *start.xyz,
                              field.seg_heading[0], field.point_heading[0], field.point_c[0],
                              0, 0, start.feed, None, None, False, None,
                              'START', True, generated_by_fillet=bool(start_fillet),
                              fillet_id=start.fillet_id, original_X=original_start[0],
                              original_Y=original_start[1], original_Z=original_start[2], emit=False))
        activation = None
        for block in blocks:
            if block.line > stroke.points[0].line:
                break
            for letter, value in block.words:
                if letter == 'M' and value in (3, 4): activation = block.line
                elif letter == 'M' and value == 5: activation = None
        prep_line = activation or stroke.points[0].line
        if activation and any(m.deposition and activation<m.line<stroke.points[0].line for m in motions):
            prep_line=stroke.points[0].line
        needs_initial_alignment = first_print_stroke and fixed_axis_start
        if needs_initial_alignment:
            initial_alignment_start_c = incoming_c if incoming_c is not None else 0.0
            initial_alignment_target_c = field.point_c[0]
            initial_alignment_xyz = start.xyz
            initial_alignment_activation_line = activation
            initial_alignment_unsafe_activation = bool(
                activation and any(
                    activation <= m.line < stroke.points[0].line and
                    (m.kind == 'RAPID' or not m.deposition) and
                    math.dist(m.start, m.end) > config.geometry.zero_length_tolerance_mm
                    for m in motions))
            if initial_alignment_unsafe_activation and not config.serialization.motion_only:
                raise ValueError(
                    'La activación M3/M4 ocurre antes del recorrido de aproximación. '
                    'Mueva M3/M4 después de llegar al punto inicial o use salida solo movimiento; '
                    'no es seguro orientar C con el proceso activo.')
        orient_after_cordon=(cordon_report['status'] in ('ADDED','ADJUSTED','KEPT') and len(fields)==2 and
                             incoming_c is not None and abs(incoming_c-field.point_c[0])>config.precision.angle_epsilon_deg)
        orient_stroke_start=(native_enabled(config) and abs((incoming_c or 0.)-field.point_c[0])>config.precision.angle_epsilon_deg)
        if needs_initial_alignment:
            orient_stroke_start=False
        already_approached=(first_print_stroke and any(
            motion.kind == 'RAPID' and
            xy_length(motion.end, start.xyz) <= config.geometry.zero_length_tolerance_mm
            for motion in motions[:int(stroke.points[0].motion_index)]))
        if (xy_length(stroke.points[0].xyz, start.xyz) > config.geometry.zero_length_tolerance_mm or orient_after_cordon or orient_stroke_start or
                (first_print_stroke and config.initial_xy_approach.enabled) or
                (first_print_stroke and xy_offset_report['applied'] and not already_approached) or
                (len(fields)==1 and stroke.layer_id==1 and config.z_conversion.enabled and
                 abs(z_report['source_layer_z_values'][0]-start.xyz[2])>config.z_conversion.z_match_tolerance_mm)):
            if orient_after_cordon:
                prep_line = stroke.points[0].line
            prep_c=(initial_alignment_start_c if needs_initial_alignment
                    else field.point_c[0])
            samples.append(Sample(len(samples) + 1, None, None, prep_line,
                                  start.motion_index - 0.5, None, -1, s_global, 0, *start.xyz,
                                  None, None, prep_c, 0, 0, start.feed, None, None,
                                  False, None, 'PREP_RAPID', False))
            if first_print_stroke and config.initial_xy_approach.enabled:
                samples[-1].generated_reason='INITIAL_XY_APPROACH'
                initial_approach_report['selected_start_xyz']=list(start.xyz)
            if orient_after_cordon: samples[-1].generated_reason='CORDON_ORIENTATION'
        if (needs_initial_alignment and
                abs(initial_alignment_target_c-initial_alignment_start_c) > config.precision.angle_epsilon_deg):
            rotation_speed=config.orientation.initial_rotation_speed_deg_s
            if config.kinematics.max_c_velocity_deg_s is not None:
                rotation_speed=min(rotation_speed,config.kinematics.max_c_velocity_deg_s)
            samples.append(Sample(
                len(samples) + 1, stroke.layer_id, stroke.id, prep_line,
                start.motion_index - 0.25, None, 0, s_global, 0, *start.xyz,
                field.seg_heading[0], field.point_heading[0], initial_alignment_target_c,
                initial_alignment_target_c-initial_alignment_start_c, 0,
                60*rotation_speed, None, None, False, None, 'C_ONLY', False,
                generated_reason='INITIAL_C_ALIGNMENT', primitive_type='C_ONLY',
                c_motion_start_deg=initial_alignment_start_c))
            initial_alignment_sample=samples[-1]
        count = 0
        sampler_input+=reference_point_counts[stroke.id]
        adaptive_rows={}
        if config.sampling.mode!='legacy':
            adaptive_rows,spans=CurveSampler(field,config).sample()
            curve_spans.extend(spans)
        for i in range(len(optimized.points) - 1):
            a, b = optimized.points[i:i + 2]
            fractions = _sample_segment(field, i, config) if config.sampling.mode=='legacy' else [r['t'] for r in adaptive_rows[i]]
            count += len(fractions)
            if count > config.sampling.max_samples_per_stroke:
                raise ValueError(f"Stroke {stroke.id}: max_samples_per_stroke superado; aumente pasos o límite")
            for fraction_index,t in enumerate(fractions):
                xyz = field.xyz_at(i,t)
                corner = field.by_vertex.get(i + 1) if t == 1 else None
                active_fillet = (fillet_by_id.get((stroke.id, b.fillet_id))
                                 if a.fillet_id is not None and a.fillet_id == b.fillet_id else None)
                raw_segment = b.original_segment if b.original_segment is not None else 0
                original = xyz if b.arc_center and not active_fillet else _original_projection(xyz, stroke, raw_segment, active_fillet)
                if b.arc_source=='FREE_CURVE_FIT':
                    original=min((_closest_on_segment(xyz,p.xyz,q.xyz) for p,q in zip(stroke.points,stroke.points[1:])),key=lambda p:math.dist(xyz,p))
                c = field.at(i, t)
                samples.append(Sample(len(samples) + 1, stroke.layer_id, stroke.id, b.line, b.motion_index,
                                      raw_segment, t, s_global + field.s[i] + t * field.seg_length[i],
                                      field.s[i] + t * field.seg_length[i], *xyz, field.seg_heading[i],
                                      field.theta_at(i,t), c,
                                      0, field.curvature_at(i,t), planned_feeds[i],
                                      None, None, bool(corner) or bool(active_fillet and t == 1 and i + 1 == active_fillet.optimized_point_end),
                                      corner.category if corner else 'FILLET' if active_fillet else None,
                                      'LINEAR', True,
                                      (motions[b.motion_index].e_start + (motions[b.motion_index].e_end - motions[b.motion_index].e_start) * t
                                       if motions[b.motion_index].e_start is not None and motions[b.motion_index].e_end is not None else None),
                                      generated_by_fillet=bool(active_fillet),
                                      fillet_id=active_fillet.id if active_fillet else b.fillet_id if t == 1 else None,
                                      original_X=original[0], original_Y=original[1], original_Z=original[2]))
                sample=samples[-1]
                sample.field_segment=i
                sample.primitive_id=f'{stroke.id}:arc:{active_fillet.id}' if active_fillet else f'{stroke.id}:line:{i}'
                sample.primitive_type='ARC' if b.arc_center else 'LINE'
                sample.feed_source=b.feed
                sample.feed_recommended=recommendations[i]['recommended_feed_mm_min']
                sample.generated_reason='FILLET' if active_fillet else 'ORIGINAL' if t==1 else 'CURVE_RESAMPLE'
                previous_t=fractions[fraction_index-1] if fraction_index>0 else 0.
                sample.orientation_error_deg=field.interpolation_error(i,previous_t,t,field.at(i,previous_t),c)
                if config.sampling.mode!='legacy':
                    row=adaptive_rows[i][fraction_index]
                    sample.geometry_error_mm=row['geometry_error_mm']
                    sample.orientation_error_deg=row['orientation_error_deg']
                    sample.primitive_type=row['primitive_type']
                    sample.source_lines=row['source_lines']
                    if not active_fillet: sample.generated_reason='ADAPTIVE_CURVE' if row['primitive_type']!='STRAIGHT' else 'STRAIGHT_ENDPOINT'
                if b.arc_source=='FREE_CURVE_FIT': sample.generated_reason='FREE_CURVE_FIT'
                if b.fit_metadata.get('reference_model')=='PARAMETRIC_HERMITE':
                    sample.curve_certified=True
                    sample.orientation_error_deg=max(sample.orientation_error_deg,b.fit_metadata['max_orientation_fit_error_deg'])
                    if b.arc_center is None:
                        sample.geometry_error_mm=max(sample.geometry_error_mm,b.fit_metadata['max_fit_error_mm'])
                if corner and corner.strategy == 'EXACT_VERTEX' and t == 1:
                    samples.append(Sample(len(samples) + 1, stroke.layer_id, stroke.id, b.line, b.motion_index,
                                          raw_segment, t, s_global + field.s[i + 1], field.s[i + 1], *xyz,
                                          field.seg_heading[i], field.point_heading[i + 1], field.point_c[i + 1],
                                          0, field.point_curvature[i + 1], b.feed, None, None, True,
                                          corner.category, 'C_ONLY', True,
                                          original_X=original[0], original_Y=original[1], original_Z=original[2]))
                    samples[-1].field_segment=i
        for rank,sample in enumerate(samples[stroke_sample_start:]):
            sample.c_reference_shift_deg=field.reference_shift_deg
            execution_order[id(sample)]=(stroke.points[1].motion_index-0.5
                                         if sample.motion_type=='PREP_RAPID' or
                                         sample.generated_reason=='INITIAL_C_ALIGNMENT'
                                         else stroke.points[1].motion_index,rank)
            if optimized.seam_reindexed and sample.deposition_active:
                sample.anchor_line=stroke.points[1].line
            if sample.feed_source is None: sample.feed_source=sample.feed_mm_min
        s_global += field.s[-1]
    trace_stages.add([asdict(s) for s in optimized_strokes],sum(len(s.points) for s in optimized_strokes))
    orientation_plan=[{'stroke_id':sid,'s':f.s,'C':f.point_c,'theta':f.point_heading,
                       'domains':[{'segment':i,'type':'ARC' if f.stroke.points[i+1].arc_center else 'CORNER' if i+1 in f.by_vertex else 'SMOOTH'} for i in range(len(f.seg_length))]}
                      for sid,f in fields.items()]
    trace_stages.add(orientation_plan,sum(len(f.point_c) for f in fields.values()))
    # Insert travel and Z-only endpoints in their source order; never orient them from a print stroke.
    endpoint_c = {}
    for sample in samples:
        if sample.emit:
            endpoint_c[sample.motion_index] = sample.C_deg
    carried_c = 0.0
    for i, motion in enumerate(motions):
        if motion.stroke_id in fields:
            carried_c = endpoint_c.get(i, carried_c)
            continue
        c = carried_c
        initial_c=carried_c
        if motion.kind=='C_ONLY' or motion.block.value('C') is not None:
            c=motion.c_end; carried_c=c
        fractions=[1.]; source_arc=None
        if motion.kind in ('ARC_CW','ARC_CCW'):
            source_arc=arc_geometry(motion); cx,cy,r,a0,sweep=source_arc
            if native_enabled(config) and config.native_arcs.source_arcs.preserve:
                steps=max(1,math.ceil(abs(math.degrees(sweep))/90-1e-10))
            else:
                chord=2*math.acos(max(-1.,1-min(config.sampling.geometry_error_mm,config.corner_smoothing.max_chord_error_mm)/r))
                steps=max(1,math.ceil(abs(sweep)/min(chord,math.radians(config.sampling.max_delta_c_deg))),
                          math.ceil(r*abs(sweep)/config.sampling.max_linear_step_mm))
            if steps>config.sampling.max_samples_per_stroke: raise ValueError('max_samples_per_stroke superado por arco travel')
            fractions=[k/steps for k in range(1,steps+1)]
        for rank,t in enumerate(fractions):
            xyz=motion.end
            arc_c=c
            if source_arc:
                xyz=motion.end if t==1 else (cx+r*math.cos(a0+sweep*t),cy+r*math.sin(a0+sweep*t),motion.start[2]+(motion.end[2]-motion.start[2])*t)
                arc_c=initial_c+(c-initial_c)*t
            samples.append(Sample(len(samples) + 1, motion.layer_id, motion.stroke_id, motion.line, i,
                                  None, t, s_global, 0, *xyz, None, None, arc_c, 0, 0,
                                  motion.feed, None, None, False, None,
                                  motion.original_kind if motion.kind == 'Z_ONLY' else motion.kind, False,
                                  motion.e_end))
            samples[-1].generated_reason='LAYER_TRANSITION' if motion.kind=='Z_ONLY' else 'TRAVEL'
            samples[-1].c_reference_shift_deg=c_reference_shift
            samples[-1].feed_source=motion.feed
            zr=z_report['motions'][i]
            if (not source_arc and xy_length(motion.start,motion.end)<=config.geometry.zero_length_tolerance_mm and
                    abs(zr['z_source']-zr['z_source_start'])<=config.geometry.zero_length_tolerance_mm and motion.e_start==motion.e_end):
                samples[-1].emit=motion.kind=='C_ONLY' and abs(motion.c_end-motion.c_start)>1e-9
            execution_order[id(samples[-1])]=(i,-1+rank/max(1,len(fractions)))
    samples.sort(key=lambda s: execution_order[id(s)])
    # A source Z-only remains Z-only at the printable seam; explicit XY travel is untouched.
    last_emitted=None
    for sample in samples:
        if sample.generated_reason=='LAYER_TRANSITION' and last_emitted is not None:
            sample.X,sample.Y=last_emitted.X,last_emitted.Y
            sample.C_deg=last_emitted.C_deg
        elif not sample.emit and sample.motion_type!='START' and last_emitted is not None:
            sample.X,sample.Y,sample.Z=last_emitted.X,last_emitted.Y,last_emitted.Z
            sample.C_deg=last_emitted.C_deg
        if sample.emit: last_emitted=sample
    if config.sampling.mode=='legacy':
        samples,simplification_report=simplify(samples,fields,config)
    else:
        output_count=sum(s.deposition_active for s in samples)
        simplification_report={'enabled':config.simplification.enabled,'input_points':sampler_input,
                              'output_points':output_count,'compression_ratio':sampler_input/max(1,output_count),
                              'max_geometry_error_mm':max((s.geometry_error_mm for s in samples if s.primitive_type!='ARC'),default=0.),
                              'max_orientation_error_deg':max((s.orientation_error_deg for s in samples),default=0.),
                              'geometry_tolerance_mm':min(config.sampling.geometry_error_mm,config.simplification.geometry_tolerance_mm),
                              'orientation_tolerance_deg':min(config.sampling.max_orientation_interp_error_deg,config.simplification.orientation_tolerance_deg)}
    global_distance = 0.0
    previous_sample = None
    for i, sample in enumerate(samples, 1):
        sample.sample_id = i
        sample.z_target=sample.Z
        sample.layer_index=sample.layer_id-1 if sample.layer_id else None
        if sample.layer_id:
            zs=z_report['source_layer_z_values'][sample.layer_id-1]
            zt=z_report['target_layer_z_values'][sample.layer_id-1]
            sample.z_source=sample.Z-(zt-zs)
            sample.z_role='DEPOSITION_Z'
            sample.z_conversion_applied=config.z_conversion.enabled and abs(zt-zs)>1e-12
            sample.z_conversion_reason='NORMALIZED_LAYER_SCHEDULE' if config.z_conversion.enabled else 'DISABLED'
        elif sample.motion_index>=0 and sample.motion_index==int(sample.motion_index):
            zr=z_report['motions'][int(sample.motion_index)]
            for key in ('z_source','z_role','z_conversion_applied','z_conversion_reason','layer_index'): setattr(sample,key,zr[key])
        if previous_sample is not None and sample.emit:
            sample.segment_length_mm=math.dist((previous_sample.X,previous_sample.Y,previous_sample.Z),(sample.X,sample.Y,sample.Z))
        sample.source_motion_id=motions[int(sample.motion_index)].source_motion_id if sample.motion_index>=0 and sample.motion_index==int(sample.motion_index) else None
        sample.source_lines=sample.source_lines or [sample.source_line]
        if sample.motion_index >= 0 and sample.motion_index == int(sample.motion_index):
            source_motion = motions[int(sample.motion_index)]
            sample.layer_closure_role = source_motion.layer_closure_role
            sample.closure_reference_line = source_motion.closure_reference_line
            sample.cordon_role = source_motion.cordon_role
            if sample.cordon_role == 'CORDON_PRIME' and sample.emit:
                sample.generated_reason = 'CORDON'
            if source_motion.closure_reference_line is not None:
                sample.source_lines = sorted(set(sample.source_lines + [source_motion.closure_reference_line]))
        sample.c_target_deg=sample.C_deg
        if sample.heading_unwrapped_deg is not None:
            theta=math.radians(sample.heading_unwrapped_deg)
            sample.tangent_xy=[math.cos(theta),math.sin(theta)]
            c=math.radians(sample.C_deg-sample.c_reference_shift_deg)
            sample.nozzle_vector_xy=[math.cos(c),math.sin(c)]
        if previous_sample is not None:
            global_distance += xy_length((previous_sample.X, previous_sample.Y, 0),
                                         (sample.X, sample.Y, 0))
        sample.s_global = global_distance
        previous_sample = sample
    trace_stages.add([s.to_dict() for s in samples],len(samples))
    kinematics, kin_warnings = enforce_acceleration(samples,config,_kinematics)
    warnings.extend(kin_warnings)
    density_report=density(samples,input_motion_count,config)
    if density_report['microblock_count'] and config.sampling.mode!='legacy':
        warnings.append(f"MICRO_BLOCK_WARNING: {density_report['microblock_count']} bloques de deposición por debajo de {config.sampling.preferred_min_block_length_mm:g} mm; conservados por tolerancia o eventos")
    trace_stages.add([s.to_dict() for s in samples],len(samples))
    max_error, per_layer, errors = _validate_sample_geometry(samples, strokes, config)
    errors.extend(validate_cordon(cordon_report, samples, config))
    errors.extend(validate_completed_closure(closure_report, samples, config))
    layer_report = _layer_report(layers, strokes, samples, per_layer, config)
    for layer in layer_report:
        index=layer['layer_id']-1
        layer.update(z_source=z_report['source_layer_z_values'][index],z_target=z_report['target_layer_z_values'][index],
                     layer_index=index,z_conversion_applied=config.z_conversion.enabled,
                     z_conversion_reason='NORMALIZED_LAYER_SCHEDULE' if config.z_conversion.enabled else 'DISABLED')
    z_errors=[abs(s.Z-z_report['target_layer_z_values'][s.layer_id-1]) for s in samples if s.layer_id and s.deposition_active]
    z_report['max_layer_z_error_mm']=max(z_errors,default=0.)
    z_report['unexpected_z_count']=sum(e>config.z_conversion.z_match_tolerance_mm for e in z_errors)
    if z_report['unexpected_z_count'] and config.z_conversion.enabled:
        warnings.append('NONPLANAR_DEPOSITION: se conserva variación Z de rampa respecto al nominal de capa')
    max_drift = max((item['layer_xy_drift_mm'] for item in layer_report), default=0.0)
    interlayer_drift, interlayer_comparisons = _interlayer_consistency(layers, strokes, optimized_strokes)
    centroid_tolerance=config.corner_smoothing.max_chord_error_mm if any(p.arc_center for s in strokes for p in s.points) else config.geometry.max_xy_deviation_mm
    if config.geometry.mode == 'PRESERVE_PATH' and max_drift > centroid_tolerance:
        errors.append(f"FAIL_GEOMETRY_DRIFT: max layer centroid drift {max_drift:.9f} mm")
    if interlayer_drift > config.geometry.max_xy_deviation_mm:
        errors.append(f"FAIL_LAYER_OFFSET: capas idénticas difieren {interlayer_drift:.9f} mm en XY")
    geometry_errors = list(errors)
    vertex_samples = {}
    samples_by_stroke = defaultdict(list)
    for sample in samples:
        if sample.stroke_id is not None and sample.emit:
            samples_by_stroke[sample.stroke_id].append(sample)
        if sample.emit and sample.original_segment is not None and sample.t == 1:
            vertex_samples[(sample.stroke_id, sample.field_segment + 1)] = sample
    for field in fields.values():
        for corner in field.corners:
            at_vertex = vertex_samples.get((corner.stroke_id, corner.point_index))
            if at_vertex:
                actual = round(at_vertex.C_deg, config.serialization.c_decimals)
                target = field.point_c[corner.point_index]
                corner.orientation_error_at_vertex_deg = abs(actual - target)
                angle_tolerance = 10 ** (-config.serialization.c_decimals) + 1e-9
                if corner.orientation_error_at_vertex_deg > angle_tolerance:
                    corner.status = 'FAIL'
                    later = (s for s in samples_by_stroke[corner.stroke_id]
                             if s.s_stroke > corner.s_vertex and abs(s.C_deg - target) <= angle_tolerance)
                    settled = next(later, None)
                    corner.settling_distance_after_vertex_mm = (
                        settled.s_stroke - corner.s_vertex if settled else
                        field.s[-1] - corner.s_vertex)
            else:
                corner.status = 'FAIL'
    corners = [corner.to_dict() for field in fields.values() for corner in field.corners]
    for corner in corners:
        if corner['status'] == 'FAIL': errors.append(f"Corner {corner['id']} en línea {corner['source_line']} inválido")
    for fillet in all_fillets:
        if fillet.status != 'PASS':
            continue
        members = [s for s in samples if s.stroke_id == fillet.stroke_id and s.fillet_id == fillet.id]
        entry = next((s for s in members if abs(s.X - fillet.t1[0]) <= 1e-7 and
                      abs(s.Y - fillet.t1[1]) <= 1e-7), None)
        exit_sample = next((s for s in reversed(members) if abs(s.X - fillet.t2[0]) <= 1e-7 and
                            abs(s.Y - fillet.t2[1]) <= 1e-7), None)
        fillet.generated_segments = sum(1 for s in members if s.generated_by_fillet and s.emit)
        if entry is None or exit_sample is None:
            fillet.status = 'FAIL'
            errors.append(f"Línea {fillet.source_line}: entrada/salida de fillet ausente")
            continue
        fillet.c_start_deg = entry.C_deg
        fillet.c_end_deg = exit_sample.C_deg
        # Functional 180° symmetry does not shorten the physical sweep of an arc.
        target = entry.C_deg + fillet.sweep_deg
        fillet.corner_orientation_error_at_arc_exit_deg = abs(
            round(exit_sample.C_deg, config.serialization.c_decimals) - target)
        angle_tolerance = 10 ** (-config.serialization.c_decimals) + 1e-9
        if fillet.corner_orientation_error_at_arc_exit_deg > angle_tolerance:
            fillet.status = 'FAIL'
            later = next((s for s in samples_by_stroke[fillet.stroke_id] if
                          s.s_stroke > exit_sample.s_stroke and abs(s.C_deg - target) <= angle_tolerance), None)
            fillet.corner_settling_distance_after_arc_mm = (
                later.s_stroke - exit_sample.s_stroke if later else
                fields[fillet.stroke_id].s[-1] - exit_sample.s_stroke)
            errors.append(f"Línea {fillet.source_line}: C no alcanza objetivo en T2")
    fillets_report = [fillet.to_dict() for fillet in all_fillets]
    for f in fillets_report:
        members=[s for s in samples if s.stroke_id==f['stroke_id'] and s.fillet_id==f['id'] and s.generated_by_fillet and s.emit]
        f.update(sample_count=len(members),legacy_sample_count=math.ceil(abs(f['sweep_deg'])/2),
                 min_segment_length_mm=min((s.segment_length_mm for s in members),default=0.),
                 max_chord_error_mm=max((s.geometry_error_mm for s in members),default=0.),
                 max_orientation_interp_error_deg=max((s.orientation_error_deg for s in members),default=0.),
                 required_block_omega_C_deg_s=max((abs(s.omega_C_deg_s) for s in members if s.omega_C_deg_s is not None),default=None))
    for item in corners:
        match=next((s for s in samples if s.stroke_id==item['stroke_id'] and s.field_segment+1==item['point_index'] and s.t==1),None)
        item['sample_id']=match.sample_id if match else None
    raw_by_id={s.id:s for s in strokes}
    for item in fillets_report:
        item['vertex_xyz']=list(raw_by_id[item['stroke_id']].points[item['point_index']].xyz)
        match=next((s for s in reversed(samples) if s.stroke_id==item['stroke_id'] and s.fillet_id==item['id']),None)
        item['sample_id']=match.sample_id if match else None
    trace = [{"index": i, "source_line": motion.line, "raw_command": motion.block.raw,
              "motion_type": motion.kind, "X": motion.end[0], "Y": motion.end[1], "Z": motion.end[2],
              "deposition": motion.deposition, "layer": motion.layer_id, "stroke": motion.stroke_id,
              "is_travel": not motion.deposition or motion.kind == 'RAPID',
              "is_z_only": motion.kind == 'Z_ONLY'} for i, motion in enumerate(motions[:30])]
    first_print_sample=next((s for s in samples if s.deposition_active),None)
    axis_labels={0:'+X',90:'+Y',180:'-X',-90:'-Y'}
    first_field=next(iter(fields.values()),None)
    effective_reference_shift=first_field.reference_shift_deg if first_field else 0.0
    initial_reference={
        "mode":initial_orientation_mode,
        "manual_alignment_required":initial_orientation_mode=='HOME_ALIGNED_TANGENT',
        "first_print_c_deg":first_print_sample.C_deg if first_print_sample else None,
        "reference_shift_deg":effective_reference_shift,
        "home_axis_deg":config.orientation.home_axis_deg if fixed_axis_start else None,
        "home_axis_label":axis_labels.get(config.orientation.home_axis_deg) if fixed_axis_start else None,
        "start_xyz":list(initial_alignment_xyz) if initial_alignment_xyz is not None else None,
        "start_c_deg":initial_alignment_start_c,
        "target_c_deg":initial_alignment_target_c,
        "rotation_delta_deg":(initial_alignment_target_c-initial_alignment_start_c
                              if initial_alignment_target_c is not None and
                              initial_alignment_start_c is not None else None),
        "world_orientation_deg":(initial_alignment_target_c-effective_reference_shift
                                  if initial_alignment_target_c is not None else None),
        "rotation_speed_deg_s":((initial_alignment_sample.feed_mm_min/60
                                 if initial_alignment_sample is not None else
                                 min(config.orientation.initial_rotation_speed_deg_s,
                                     config.kinematics.max_c_velocity_deg_s)
                                 if fixed_axis_start and config.kinematics.max_c_velocity_deg_s is not None
                                 else config.orientation.initial_rotation_speed_deg_s)
                                if fixed_axis_start else None),
        "rotation_feed_deg_min":(initial_alignment_sample.feed_mm_min
                                 if initial_alignment_sample is not None else None),
        "sample_id":initial_alignment_sample.sample_id if initial_alignment_sample is not None else None,
        "required":initial_alignment_sample is not None,
        "status":('GENERATED' if initial_alignment_sample is not None else
                  'ALREADY_ALIGNED' if fixed_axis_start and first_print_sample is not None else
                  'NOT_APPLICABLE'),
        "xyz_held":True if fixed_axis_start and first_print_sample is not None else None,
        "before_deposition":(initial_alignment_sample.sample_id < first_print_sample.sample_id
                             if initial_alignment_sample is not None and first_print_sample is not None
                             else True if fixed_axis_start and first_print_sample is not None else None),
        "deposition_active":False if fixed_axis_start and first_print_sample is not None else None,
        "exact_stop_enforced":initial_alignment_sample is not None,
        "source_activation_line":initial_alignment_activation_line,
        "source_activation_before_approach":initial_alignment_unsafe_activation,
        "material_control":('EXTERNAL_MANUAL' if config.serialization.motion_only
                            else 'SOURCE_PROCESS_COMMANDS')}
    report = {"version": "3.2", "backend_version":BACKEND_VERSION,"input_hash":input_hash,
              "config_hash":config_hash,"conversion_id":conversion_id,"config":config.to_dict(),
              "z_conversion":z_report,"curve_spans":curve_spans,"density":density_report,
              "layer_completion":closure_report,
              "open_path_transitions":open_transition_report,
              "cordon":cordon_report,
              "initial_xy_approach":initial_approach_report,
              "xy_boundary_offset":xy_offset_report,
              "linuxcnc":{'policy':config.linuxcnc.path_mode,'source_modes':path_modes,'machine_mode_verified':False},
              "safe_seams":safe_seams,"simplification":simplification_report,
              "initial_c_reference":initial_reference,
              "feed_plan":{'policy':config.process.feed_policy,'primitives':feed_rows,
                           'unknown_feed_segments':sum(row['feed_source'] is None for row in feed_rows),
                           'final_feeds':[{'sample_id':s.sample_id,'source':s.feed_source,'planned':s.feed_mm_min,'recommended':s.feed_recommended} for s in samples if s.deposition_active]},
              "status": 'FAIL' if errors else 'PASS WITH WARNINGS' if warnings else 'PASS',
              "geometry": {"status": 'FAIL' if geometry_errors else 'PASS',
                           "mode": config.geometry.mode,
                           "modified_locally": any(f.status == 'PASS' for f in all_fillets),
                           "max_xy_deviation_mm": max_error,
                           "max_path_deviation_mm": max(max_error, simplification_report['max_geometry_error_mm'], max((f.max_path_deviation_mm for f in all_fillets
                                                                          if f.status == 'PASS'), default=0.0)),
                           "max_layer_xy_drift_mm": max_drift,
                           "max_interlayer_xy_drift_mm": interlayer_drift,
                           "interlayer_comparisons": interlayer_comparisons},
              "layers": layer_report,
              "strokes": [{"stroke_id": s.id, "layer_id": s.layer_id, "deposition": s.deposition,
                           "points": len(s.points), "source_line_start": s.points[0].line,
                           "source_line_end": s.points[-1].line} for s in strokes],
              "corners": corners,
              "fillets": fillets_report,
              "orientation": {"status": 'FAIL' if any(c['status']=='FAIL' for c in corners) or
                              any(f['status']=='FAIL' for f in fillets_report) else 'PASS',
                              "max_corner_error_at_vertex_deg": max((c['orientation_error_at_vertex_deg'] for c in corners), default=0),
                              "max_corner_settling_mm": max((c['settling_distance_after_vertex_mm'] for c in corners), default=0),
                              "max_fillet_exit_error_deg": max((f['corner_orientation_error_at_arc_exit_deg'] for f in fillets_report), default=0),
                              "max_fillet_settling_mm": max((f['corner_settling_distance_after_arc_mm'] for f in fillets_report), default=0)},
              "kinematics": kinematics, "warnings": warnings, "errors": errors,
              "initial_trace": trace,
              "initial_sample_trace": [{"index": i, "source_line": sample.source_line,
                                        "motion_type": sample.motion_type,
                                        "X": sample.X, "Y": sample.Y, "Z": sample.Z,
                                        "C_deg": sample.C_deg,
                                        "deposition": sample.deposition_active,
                                        "layer": sample.layer_id, "stroke": sample.stroke_id,
                                        "is_travel": not sample.deposition_active,
                                        "is_z_only": (sample.motion_index >= 0 and
                                                      sample.motion_index == int(sample.motion_index) and
                                                      motions[int(sample.motion_index)].kind == 'Z_ONLY')}
                                       for i, sample in enumerate(samples[:30])],
              "counts": {"source_lines": len(blocks), "motions": len(motions), "layers": len(layers),
                         "strokes": len(strokes), "samples": len(samples),
                         "fillets": sum(f.status == 'PASS' for f in all_fillets)}}
    printable=tuple((s.id,s.layer_id,s.deposition,tuple(p.xyz for p in s.points)) for s in optimized_strokes)
    primitives=build_machine_primitives(fields,motions,samples,config)
    native_metrics,native_warnings=analyze_primitives(primitives,config)
    for fillet in fillets_report:
        fillet['display_sample_count']=fillet['sample_count']
        fillet['machine_block_count']=sum(p.stroke_id==fillet['stroke_id'] and p.metadata.get('fillet_id')==fillet['id']
                                          for p in primitives if p.type=='ARC') if native_enabled(config) else fillet['sample_count']
    warnings.extend(native_warnings)
    if native_enabled(config):
        density_report=primitive_density(primitives,input_motion_count,config)
        report['density']=density_report
        kinematics['max_omega_c_deg_s']=native_metrics['max_required_omega_c_deg_s']
    if any(d['output']=='G1_FALLBACK' for d in arc_fit_decisions):
        warnings.append('ARC_FIT_FALLBACK: algunos tramos conservan G1 adaptativo por tolerancias o eventos')
    report['native_arcs']=dict(native_metrics,enabled=native_enabled(config),decisions=arc_fit_decisions,
                               primitives=[p.to_dict() for p in primitives if p.type=='ARC'])
    reconstructed=[p for p in primitives if p.metadata.get('reference_model')=='PARAMETRIC_HERMITE']
    report['curve_reconstruction']={
        'requested_model':config.arc_fit.reference_model,'applied':bool(reconstructed),
        'orientation_reference':'PARAMETRIC_TANGENT' if reconstructed else 'POLYLINE_PCHIP',
        'legacy_angular_equivalence_claimed':False,
        'max_certified_geometry_error_mm':max((p.metadata['max_fit_error_mm'] for p in reconstructed),default=0.),
        'max_certified_orientation_error_deg':max((p.metadata['max_orientation_fit_error_deg'] for p in reconstructed),default=0.),
        'max_reconstruction_error_mm':max((p.metadata.get('reconstruction_error_mm',0.) for p in reconstructed),default=0.),
        'native_arc_count':sum(p.type=='ARC' for p in reconstructed),'line_count':sum(p.type=='LINE' for p in reconstructed),
        'note':'C sigue la tangente de la curva reconstruida; el límite angular se refiere a esa curva, no al PCHIP anterior.'
               if reconstructed else 'Se conserva el modelo de referencia anterior.',
        'source_function_recovered':False,'optimization_enabled':False}
    if reconstructed:
        report['geometry']['modified_locally']=True
        report['geometry']['max_path_deviation_mm']=max(report['geometry']['max_path_deviation_mm'],
                                                        report['curve_reconstruction']['max_certified_geometry_error_mm'])
    elif config.arc_fit.reference_model=='PARAMETRIC_HERMITE':
        reasons=[d['split_reason'] for d in arc_fit_decisions if d.get('split_reason') and d['output']=='G1_FALLBACK']
        report['curve_reconstruction']['not_applied_reasons']=reasons or [
            'PRESERVE_PATH conserva la referencia anterior' if config.geometry.mode=='PRESERVE_PATH' else
            'Opciones desactivadas, eventos protegidos o tramo insuficiente para reconstruir']
    final=FinalMotionPlan(conversion_id,tuple(samples),design_path,printable,primitives)
    try: gcode,serialization_map = _serialize(blocks,motions,final,config) if not errors else ('',[])
    except ValueError as error:
        errors.append('ARC_SERIALIZATION: '+str(error)); gcode,serialization_map='',[]
    report['serialization_map']=serialization_map
    if not errors:
        report['validators']=validate_final(samples,gcode,serialization_map,report,config,primitives)
        failed=[name for name,value in report['validators'].items() if value['status']=='FAIL']
        if failed:
            errors.append('Validadores fallidos: '+', '.join(failed)); gcode=''
    else:
        report['validators']={name:{'status':'FAIL' if name in ('Geometry','Orientation') else 'NOT CONFIGURED'}
                              for name in ('Syntax','Semantic','Geometry','Corners','Curves','Layers','Orientation','Kinematics','Sampling','Serialization')}
    report['status']='FAIL' if errors else 'PASS WITH WARNINGS' if warnings else 'PASS'
    trace_stages.add(gcode,len(gcode.splitlines()))
    report['stages']=trace_stages.stages
    import re
    report['issues']=[]
    for severity,messages in (('FAIL',errors),('WARNING',warnings)):
        for message in messages:
            source_match=re.search(r'[Ll]ínea (\d+)',message)
            line=int(source_match[1]) if source_match else None
            related=next((s for s in samples if s.source_line==line),None) if line is not None else None
            report['issues'].append({'status':severity,'message':message,'source_line':line,
                                     'sample_id':related.sample_id if related else None})
    report['metrics']={'input_motion_count':input_motion_count,'output_motion_count':sum(s.emit for s in samples),
        'compression_ratio':simplification_report['compression_ratio'],
        'corner_count':len(corners)+len(all_fillets),'fillet_count':report['counts']['fillets'],
        'max_geometry_deviation_mm':report['geometry']['max_path_deviation_mm'],
        'max_fillet_polyline_deviation_mm':max((f.max_path_deviation_mm for f in all_fillets if f.status=='PASS'),default=0.),
        'max_fillet_vertex_setback_mm':max((f['vertex_setback_mm'] for f in fillets_report if f['status']=='PASS'),default=0.),
        'max_orientation_error_deg':max((s.orientation_error_deg for s in samples),default=0.),
        'max_delta_c_deg':max((abs(s.delta_C_deg) for s in samples),default=0.),
        'max_omega_c_deg_s':kinematics['max_omega_c_deg_s'],'max_alpha_c_deg_s2':kinematics['max_alpha_c_deg_s2'],
        'layer_count':len(layers),'max_layer_xy_drift_mm':interlayer_drift,
        'safe_seam_status':'WARNING' if any(s['status']=='WARNING' for s in safe_seams) else 'PASS' if any(s['status']=='PASS' for s in safe_seams) else 'NOT CONFIGURED',
        'nan_count':sum(math.isnan(v) for s in samples for v in (s.X,s.Y,s.Z,s.C_deg)),
        'infinity_count':sum(math.isinf(v) for s in samples for v in (s.X,s.Y,s.Z,s.C_deg))}
    report['metrics'].update(output_G1_count=density_report['output_G1_count'],
                             output_G2_count=sum(p.type=='ARC' and p.direction=='CW' for p in primitives),
                             output_G3_count=sum(p.type=='ARC' and p.direction=='CCW' for p in primitives),
                             native_arc_count=native_metrics['native_arc_count'],fitted_arc_count=native_metrics['fitted_arc_count'],
                             file_size_bytes=len(gcode.encode('utf-8')),
                             native_compression_ratio=input_motion_count/max(1,len(primitives)))
    report['metrics'].update(input_counts)
    display_samples=final.samples
    simulation = {"samples": [s.to_dict() for s in display_samples],
                  "visualization_stride": 1,
                  "full_sample_count": len(samples),
                  "program_start_index": 0,
                  "initial_orientation_start_index": next((i for i, s in enumerate(display_samples)
                                                            if s.generated_reason=='INITIAL_C_ALIGNMENT'),
                                                           next((i for i, s in enumerate(display_samples)
                                                                 if s.deposition_active), 0)),
                  "deposition_start_index": next((i for i, s in enumerate(display_samples)
                                                  if s.deposition_active), 0),
                  "original_strokes": design_strokes,
                  "optimized_strokes": [{"stroke_id": s.id, "layer_id": s.layer_id,
                                         "deposition": s.deposition,
                                         "points": [list(p.xyz) for p in s.points]} for s in optimized_strokes],
                  "corners": corners, "fillets": fillets_report}
    layer_transitions=[]
    previous_emitted=None
    final_strokes=[]; current=None
    for s in samples:
        if s.generated_reason=='LAYER_TRANSITION':
            layer_transitions.append({'sample_id':s.sample_id,'source_line':s.source_line,'xyz':[s.X,s.Y,s.Z],
                                      'start_xyz':list(previous_emitted) if previous_emitted else None})
        if s.emit or s.motion_type=='START' or s.display_only:
            key=(s.stroke_id,s.layer_id,s.deposition_active)
            if current is None or (current['stroke_id'],current['layer_id'],current['deposition'])!=key or s.motion_type=='START':
                current={'stroke_id':s.stroke_id,'layer_id':s.layer_id,'deposition':s.deposition_active,
                         'cordon':s.cordon_role=='CORDON_PRIME','points':[],'sample_ids':[]}
                final_strokes.append(current)
                if s.emit and previous_emitted and s.motion_type!='START': current['points'].append(list(previous_emitted))
            current['points'].append([s.X,s.Y,s.Z]); current['sample_ids'].append(s.sample_id)
        if s.emit: previous_emitted=(s.X,s.Y,s.Z)
    simulation.update(schema_version='3.2',conversion_id=conversion_id,input_hash=input_hash,config_hash=config_hash,
                      backend_version=BACKEND_VERSION,design_path=simulation['original_strokes'],
                      printable_path=simulation['optimized_strokes'],classified_geometry=classifications,
                      layers=layer_report,layer_transitions=layer_transitions,safe_seams=safe_seams,
                      final_strokes=final_strokes,
                      orientation=report['orientation'],feed_plan=report['feed_plan'],audit=report,
                      stages=trace_stages.stages,raw_program=source,
                      parsed_program=parsed_program,canonical_motion_program=canonical_program,
                      orientation_plan=orientation_plan,process_plan=report['feed_plan']['final_feeds'],
                      tolerances={'geometry':config.to_dict()['geometry'],'sampling':config.to_dict()['sampling'],
                                  'precision':config.to_dict()['precision']},
                      final_motion_plan={'sample_ids':[s.sample_id for s in final.samples],
                                         'emitted_sample_ids':[s.sample_id for s in final.samples if s.emit]})
    # Analytic primitive endpoints remain in optimized_strokes. The drawable
    # printable path is the backend's final G1 tessellation of those primitives.
    simulation['printable_path']=[p for p in final_strokes if p['deposition']]
    simulation['arc_primitives']=[{'stroke_id':s.id,'layer_id':s.layer_id,'start_xyz':list(a.xyz),
                                  'end_xyz':list(b.xyz),'center':b.arc_center,'radius':b.arc_radius,
                                  'sweep_deg':b.arc_sweep_deg,'source_line':b.line,'fillet_id':b.fillet_id}
                                 for s in optimized_strokes for a,b in zip(s.points,s.points[1:]) if b.arc_center is not None]
    simulation['machine_primitives']=[p.to_dict() for p in primitives]
    simulation['final_motion_plan']['primitive_ids']=[p.id for p in primitives]
    simulation['arc_primitives']=[p.to_dict() for p in primitives if p.type=='ARC']
    return Result(samples, gcode, report, _csv(samples), simulation,final)
