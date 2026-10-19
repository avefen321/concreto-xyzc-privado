"""A separate sacrificial first-layer primer selected before layer completion.

The source may contain a long implicit G1 approach. Only a geometrically
confirmed entry is treated as that primer. M/E/S process programs are left
unchanged because their material state cannot be inferred from this movement.
"""
from dataclasses import replace
import math

from .gcode import arc_geometry


def _xy_distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def _signed_area(points):
    return sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(points, points[1:]+points[:1]))/2


def _first_contour(motions, first, candidate, ztol, xytol):
    z = motions[first].end[2]
    start = max(first, candidate+1) if candidate is not None else first
    result = []
    for motion in motions[start:]:
        if abs(motion.end[2]-z) > ztol or abs(motion.start[2]-z) > ztol:
            break
        if motion.deposition and motion.kind != 'RAPID':
            # A modal G1 may repeat the position reached by the preceding G0.
            # It starts no printable direction and must not hide the first real
            # segment. Native full-circle arcs remain valid even when XY closes.
            if (motion.kind not in ('ARC_CW', 'ARC_CCW') and
                    _xy_distance(motion.start, motion.end) <= xytol):
                continue
            result.append(motion)
        elif result:
            break
    return result


def analyze_cordon(motions, blocks, config, closure_review):
    """Return a preview usable by both the UI and the conversion decision."""
    report = {'mode': config.cordon.mode, 'status': 'UNREVIEWED',
              'length_mm': config.cordon.length_mm, 'reason': '',
              'existing': False, 'existing_length_mm': 0.,
              'source_entry_length_mm': 0.,
              'can_add': False, 'can_keep': False, 'can_omit': False,
              'first_motion_index': None, 'entry_motion_index': None,
              'preview_start_xyz': None, 'contour_start_xyz': None,
              'source_line': None, 'preview_direction': None,
              'travel_to_retarget_index': None,
              'negative_xy': False, 'outside_source_bbox': False}
    dep = [i for i, m in enumerate(motions) if m.deposition and m.kind != 'RAPID']
    if not dep:
        report['reason'] = 'No se encontró un inicio de deposición.'
        return report
    first = dep[0]
    candidate = closure_review.get('approach_motion_index') if closure_review.get('review_required') else None
    tol = config.layer_completion.match_tolerance_mm
    ztol = config.layers.z_change_tolerance_mm
    # A first G1 ending at a closed loop seam is also an approach, as in the
    # six-layer square. The movement from implicit origin is not loop geometry.
    if candidate is None and motions[first].kind == 'LINEAR' and _xy_distance(motions[first].start, motions[first].end) > tol:
        seam = motions[first].end
        run = _first_contour(motions, first, first, ztol, tol)
        points = [seam[:2]]+[m.end[:2] for m in run]
        if (len(points) >= 4 and _xy_distance(points[-1], seam) <= tol and
                abs(_signed_area(points[:-1])) > tol*tol):
            candidate = first
    contour = _first_contour(motions, first, candidate, ztol, tol)
    seam = motions[candidate].end if candidate is not None else motions[first].start
    report.update(first_motion_index=first, entry_motion_index=candidate,
                  contour_start_xyz=list(seam), source_line=motions[candidate if candidate is not None else first].line)
    # With no detected approach, choosing "omit" is always a no-op. It must
    # remain available even if a malformed or degenerate contour prevents us
    # from constructing the optional added cordon preview.
    if candidate is None:
        report['can_omit'] = True
    if candidate is not None:
        source = motions[candidate]
        length = _xy_distance(source.start, source.end)
        dx, dy = abs(source.end[0]-source.start[0]), abs(source.end[1]-source.start[1])
        report['source_entry_length_mm'] = length
        report['existing'] = 10 <= length <= 100 and min(dx, dy)/length >= 0.25
        report['existing_length_mm'] = length if report['existing'] else 0.
        report['can_keep'] = report['existing']
    has_e = any(b.value('E') is not None for b in blocks)
    m_process = [b for b in blocks if any(k == 'M' and v in (3, 4, 5) for k, v in b.words)]
    other_process = any(any(k == 'M' and v not in (2, 3, 4, 5, 30) for k, v in b.words) or
                        (b.value('S') is not None and not any(k == 'M' and v in (3, 4) for k, v in b.words))
                        for b in blocks)
    activation = next((b for b in reversed(m_process) if b.line < motions[first].line and
                       any(k == 'M' and v in (3, 4) for k, v in b.words)), None)
    safe_preprocess_travel = None
    if activation is not None:
        preceding = [(i, m) for i, m in enumerate(motions[:first]) if m.line < activation.line]
        if preceding:
            i, travel = preceding[-1]
            entry_start = motions[candidate].start if candidate is not None else seam
            if (travel.kind == 'RAPID' and not travel.deposition and
                    _xy_distance(travel.end, entry_start) <= tol and
                    abs(travel.end[2]-entry_start[2]) <= ztol and
                    not any(m.line > travel.line and m.line < motions[first].line for m in motions[:first])):
                safe_preprocess_travel = i
    explicit_process = bool(m_process)
    unsafe_process = (has_e or other_process or
                      explicit_process and (safe_preprocess_travel is None or activation is None or
                                            any(b.line > activation.line and b.line < motions[first].line and
                                                any(k == 'M' and v == 5 for k, v in b.words) for b in m_process)))
    if unsafe_process or any('LOCKED' in m.block.raw.upper() for m in motions[:first+1]):
        report['can_keep'] = True
        report['can_omit'] = not report['existing'] and candidate is None
        report['reason'] = 'No se puede reubicar el cordón con el estado de material o el viaje previo de este G-code.'
        return report
    if explicit_process:
        report['travel_to_retarget_index'] = safe_preprocess_travel
        report['can_omit'] = True
    if not contour:
        report['reason'] = 'No se pudo determinar la dirección inicial del contorno.'
        return report
    leading = contour[0]
    if leading.kind in ('ARC_CW', 'ARC_CCW'):
        cx, cy, _, angle, sweep = arc_geometry(leading)
        tangent = (-math.sin(angle)*math.copysign(1., sweep), math.cos(angle)*math.copysign(1., sweep))
    else:
        length = _xy_distance(leading.start, leading.end)
        if length <= tol:
            report['reason'] = 'No se pudo determinar una tangente no nula.'
            return report
        tangent = ((leading.end[0]-leading.start[0])/length,
                   (leading.end[1]-leading.start[1])/length)
    points = [seam[:2]] + [m.end[:2] for m in contour]
    winding = _signed_area(points)
    if abs(winding) > tol*tol:
        normal = (tangent[1], -tangent[0]) if winding > 0 else (-tangent[1], tangent[0])
        direction = (normal[0]-tangent[0], normal[1]-tangent[1])
    else:
        # For a genuinely open path there is no polygon exterior. Place the
        # visible primer behind and to one side of the first print direction.
        direction = (tangent[1]-tangent[0], -tangent[0]-tangent[1])
    dx = -1 if direction[0] < -1e-9 else 1
    dy = -1 if direction[1] < -1e-9 else 1
    step = config.cordon.length_mm/math.sqrt(2)
    start = (seam[0]+dx*step, seam[1]+dy*step, seam[2])
    all_xy = [p[:2] for m in motions for p in (m.start, m.end)]
    bbox = (min(p[0] for p in all_xy), max(p[0] for p in all_xy),
            min(p[1] for p in all_xy), max(p[1] for p in all_xy))
    report.update(status='AVAILABLE', can_add=True, can_omit=True,
                  preview_start_xyz=list(start), preview_direction=('N' if dy > 0 else 'S')+('E' if dx > 0 else 'O'),
                  negative_xy=start[0]<0 or start[1]<0,
                  outside_source_bbox=start[0]<bbox[0] or start[0]>bbox[1] or start[1]<bbox[2] or start[1]>bbox[3],
                  reason='Inicio del contorno y dirección de entrada identificados.')
    return report


def prepare_cordon(motions, blocks, config, closure_review):
    """Apply a selected approach on canonical motions, before layer building."""
    report = analyze_cordon(motions, blocks, config, closure_review)
    mode = config.cordon.mode
    if mode == 'legacy':
        report['status'] = 'LEGACY'
        return motions, report
    candidate = report['entry_motion_index']
    first = report['first_motion_index']
    if mode == 'keep' and not report['can_add']:
        report['status'] = 'PRESERVED_SOURCE'
        return motions, report
    if mode == 'add' and not report['can_add'] or mode == 'omit' and not report['can_omit']:
        raise ValueError('No se puede modificar el cordón: '+report['reason'])
    if mode == 'keep' and not report['can_keep']:
        if candidate is not None and report['can_omit']:
            source = motions[candidate]
            output = list(motions)
            output[candidate] = replace(source, kind='RAPID', original_kind='RAPID', deposition=False,
                                        cordon_role='CORDON_OMITTED_TRAVEL')
            report['status'] = 'NO_EXISTING'
            return output, report
        report['status'] = 'NO_EXISTING'
        return motions, report
    if mode == 'omit' and candidate is None:
        report['status'] = 'ABSENT'
        return motions, report
    if mode == 'omit':
        source = motions[candidate]
        output = list(motions)
        if report['travel_to_retarget_index'] is not None:
            j = report['travel_to_retarget_index']
            output[j] = replace(output[j], end=tuple(report['contour_start_xyz']),
                                cordon_role='CORDON_OMITTED_TRAVEL')
            del output[candidate]
        else:
            output[candidate] = replace(source, kind='RAPID', original_kind='RAPID', deposition=False,
                                        cordon_role='CORDON_OMITTED_TRAVEL')
        report['status'] = 'OMITTED'
        return output, report
    if mode == 'keep':
        source = motions[candidate]
        output = list(motions)
        output[candidate] = replace(source, kind='LINEAR', original_kind='LINEAR', deposition=True,
                                    cordon_role='CORDON_PRIME')
        output[candidate+1:] = [replace(m, barrier_id=m.barrier_id+1) for m in output[candidate+1:]]
        report.update(status='KEPT', actual_start_xyz=list(source.start),
                      actual_end_xyz=list(source.end), actual_length_mm=report['existing_length_mm'])
        return output, report
    # Add/adjust: an explicit G0 reaches the 45-degree point, followed by one
    # G1 primer into the original seam. A barrier isolates the printed contour.
    start = tuple(report['preview_start_xyz'])
    seam = tuple(report['contour_start_xyz'])
    output = list(motions)
    if candidate is not None:
        source = motions[candidate]
        prime = replace(source, start=start, end=seam, kind='LINEAR', original_kind='LINEAR', deposition=True,
                        cordon_role='CORDON_PRIME')
        if report['travel_to_retarget_index'] is not None:
            j = report['travel_to_retarget_index']
            output[j] = replace(output[j], end=start, cordon_role='CORDON_TRAVEL')
            output[candidate] = prime
            boundary = candidate+1
        else:
            travel = replace(source, end=start, kind='RAPID', original_kind='RAPID', deposition=False,
                             cordon_role='CORDON_TRAVEL')
            output[candidate:candidate+1] = [travel, prime]
            boundary = candidate+2
        status = 'ADJUSTED' if report['existing'] else 'ADDED'
    else:
        source = motions[first]
        prime = replace(source, start=start, end=seam, kind='LINEAR', original_kind='LINEAR', deposition=True,
                        arc={}, cordon_role='CORDON_PRIME')
        if report['travel_to_retarget_index'] is not None:
            j = report['travel_to_retarget_index']
            output[j] = replace(output[j], end=start, cordon_role='CORDON_TRAVEL')
            output[first:first] = [prime]
            boundary = first+1
        else:
            travel = replace(source, start=seam, end=start, kind='RAPID', original_kind='RAPID', deposition=False,
                             arc={}, cordon_role='CORDON_TRAVEL')
            output[first:first] = [travel, prime]
            boundary = first+2
        status = 'ADDED'
    output[boundary:] = [replace(m, barrier_id=m.barrier_id+1) for m in output[boundary:]]
    report.update(status=status, actual_start_xyz=list(start), actual_end_xyz=list(seam),
                  actual_length_mm=config.cordon.length_mm)
    return output, report


def validate_cordon(report, samples, config):
    if report['status'] not in ('ADDED', 'ADJUSTED', 'KEPT'):
        return []
    prime = [s for s in samples if s.cordon_role == 'CORDON_PRIME']
    emitted = [s for s in prime if s.emit and s.deposition_active]
    if not emitted:
        report['validation'] = {'status': 'FAIL', 'reason': 'Faltan muestras de cordón.'}
        return ['FAIL_CORDON: faltan muestras de cordón.']
    first = prime[0]
    last = emitted[-1]
    actual = math.hypot(last.X-first.X, last.Y-first.Y)
    target = report['actual_length_mm']
    gap = math.hypot(last.X-report['actual_end_xyz'][0], last.Y-report['actual_end_xyz'][1])
    after = next((s for s in samples if s.emit and s.deposition_active and s.stroke_id != first.stroke_id and
                  s.layer_id == first.layer_id), None)
    main_start = next((s for s in samples if after is not None and s.motion_type == 'START' and
                       s.stroke_id == after.stroke_id), None)
    starts_at_seam = (main_start is not None and
                      math.hypot(main_start.X-last.X,main_start.Y-last.Y) <= config.geometry.max_xy_deviation_mm and
                      abs(main_start.Z-last.Z) <= config.geometry.max_xy_deviation_mm)
    physical_c_before_main = next((s.C_deg for s in samples if s.generated_reason == 'CORDON_ORIENTATION' and
                                   s.emit and math.hypot(s.X-last.X,s.Y-last.Y) <= config.geometry.max_xy_deviation_mm),
                                  last.C_deg)
    orientation_ready = (main_start is not None and
                         abs(physical_c_before_main-main_start.C_deg) <= config.precision.angle_epsilon_deg)
    okay = (abs(actual-target) <= config.geometry.max_xy_deviation_mm and
            gap <= config.geometry.max_xy_deviation_mm and starts_at_seam and orientation_ready)
    report['validation'] = {'status': 'PASS' if okay else 'FAIL', 'actual_length_mm': actual,
                            'seam_gap_mm': gap, 'separate_stroke': starts_at_seam,
                            'orientation_ready_at_seam': orientation_ready,
                            'orientation_change_deg': abs(last.C_deg-main_start.C_deg) if main_start else None,
                            'stroke_id': first.stroke_id, 'layer_id': first.layer_id}
    return [] if okay else ['FAIL_CORDON: el tramo inicial no conserva longitud, seam o trazo independiente.']
