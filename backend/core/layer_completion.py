"""Review repeated planar contours before interpreting diagonal layer entries.

Only the proven pattern of a stable seam and a repeated G1 XY/Z return is
repairable. Its XY geometry is already in the source; no arbitrary closing
chord is invented. Explicit process/volume, ramps and ambiguous paths are left
alone. The frontend asks before setting the complete decision.
"""
from dataclasses import replace
import math
from .gcode import parse, interpret, arc_geometry


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _primitives(motions, tolerance):
    """Compare ordered geometry, allowing collinear or co-circular subdivision."""
    result = []
    for m in motions:
        if m.kind == 'LINEAR' and _distance(m.start, m.end) <= tolerance:
            continue
        p = {'kind': m.kind, 'start': m.start[:2], 'end': m.end[:2]}
        if m.kind in ('ARC_CW', 'ARC_CCW'):
            cx, cy, r, a, sweep = arc_geometry(m)
            p.update(center=(cx, cy), radius=r, angle=a, sweep=sweep)
        if result and result[-1]['kind'] == p['kind']:
            prior = result[-1]
            if p['kind'] == 'LINEAR':
                a, b = prior['start'], prior['end']
                c = p['end']
                u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
                # Subdivision may differ, but turning/reversing a segment may not.
                merge = (u[0]*v[0]+u[1]*v[1] > 0 and
                         abs(u[0]*v[1]-u[1]*v[0]) / max(math.hypot(*u), math.hypot(*v)) <= tolerance)
            else:
                merge = (_distance(prior['center'], p['center']) <= tolerance and
                         abs(prior['radius'] - p['radius']) <= tolerance)
            if merge:
                prior['end'] = p['end']
                if 'sweep' in prior: prior['sweep'] += p['sweep']
                continue
        result.append(p)
    return result


def _same_path(a, b, tolerance):
    if len(a) != len(b): return False
    for p, q in zip(a, b):
        if (p['kind'] != q['kind'] or _distance(p['start'], q['start']) > tolerance or
                _distance(p['end'], q['end']) > tolerance):
            return False
        if 'center' in p and (_distance(p['center'], q['center']) > tolerance or
                             abs(p['radius']-q['radius']) > tolerance or
                             abs(p['sweep']-q['sweep'])*p['radius'] > tolerance):
            return False
    return True


def _area(primitives):
    points = [primitives[0]['start']]
    for p in primitives:
        if 'center' in p:
            steps = max(2, math.ceil(abs(p['sweep'])/(math.pi/8)))
            points.extend((p['center'][0]+p['radius']*math.cos(p['angle']+p['sweep']*i/steps),
                           p['center'][1]+p['radius']*math.sin(p['angle']+p['sweep']*i/steps))
                          for i in range(1, steps))
        points.append(p['end'])
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(points, points[1:]+points[:1]))/2)


def analyze_closure(motions, blocks, config):
    report = {'review_required': False, 'status': 'NOT_APPLICABLE',
              'decision': config.layer_completion.decision, 'reason': '',
              'match_tolerance_mm': config.layer_completion.match_tolerance_mm,
              'reference_layers': [], 'layers': [], 'affected_layer_count': 0,
              'missing_first_layer_length_mm': 0., 'added_length_mm': 0.,
              'reassigned_return_count': 0, 'added_closure_count': 0}
    def skip(reason):
        report['reason'] = reason
        return report
    if not motions: return skip('Sin movimientos.')
    tol = config.layer_completion.match_tolerance_mm
    ztol = config.layers.z_change_tolerance_mm
    active = [i for i, m in enumerate(motions) if m.deposition and m.kind != 'RAPID']
    if not active: return skip('Sin deposición.')
    first, last = active[0], active[-1]
    span = motions[first:last+1]
    if any(not m.deposition or m.kind not in ('LINEAR', 'ARC_CW', 'ARC_CCW') for m in span):
        return skip('Varios trazos o movimientos de viaje entre contornos; cierre ambiguo.')
    # Do not infer that a commanded extrusion/process-on move was an approach.
    if any(b.value('E') is not None or any(k == 'M' and v not in (2, 30) for k, v in b.words)
           or b.value('S') is not None for b in blocks if b.line <= motions[last].line):
        return skip('Proceso o volumen explícito: no se infiere ni se redistribuye deposición.')
    if len({m.barrier_id for m in span}) != 1 or any('LOCKED' in m.block.raw.upper() for m in span):
        return skip('Eventos bloqueados o cambios de estado entre capas.')
    groups = [{'indices': [first], 'entry': None, 'z': motions[first].end[2]}]
    for i in range(first+1, last+1):
        m = motions[i]
        dz = m.end[2]-m.start[2]
        if abs(dz) > ztol:
            if m.kind != 'LINEAR' or dz <= 0:
                return skip('Rampa, hélice o descenso: no es una entrada de capa confirmada.')
            groups.append({'indices': [], 'entry': i, 'z': m.end[2]})
        else:
            groups[-1]['indices'].append(i)
    if len(groups)-1 < config.layer_completion.min_reference_layers:
        return skip('Se requieren al menos dos capas posteriores de referencia.')
    if any(len(g['indices']) < 2 for g in groups):
        return skip('Capas sin un contorno plano suficiente; posible rampa.')
    refs = groups[1:]
    seam = motions[refs[0]['entry']].end[:2]
    first_move = motions[first]
    approach = None
    if _distance(first_move.start, seam) <= tol and abs(first_move.start[2]-groups[0]['z']) <= ztol:
        start = first_move.start[:2]
    elif first_move.kind == 'LINEAR' and _distance(first_move.end, seam) <= tol:
        approach = first
        start = first_move.end[:2]
        groups[0]['indices'] = groups[0]['indices'][1:]
    else:
        return skip('No se puede distinguir el inicio del contorno de la aproximación.')
    end = motions[groups[0]['indices'][-1]].end[:2]
    gap = _distance(start, end)
    if gap <= tol:
        report['status'] = 'NOT_NEEDED'
        return skip('La primera capa ya regresa a su inicio; no necesita cierre.')
    base = _primitives([motions[i] for i in groups[0]['indices']], tol)
    if not base or _area(base) <= tol*tol:
        return skip('Contorno degenerado o trayectoria abierta sin área.')
    for g in groups:
        run = [motions[i] for i in g['indices']]
        if any(abs(m.start[2]-g['z']) > ztol or abs(m.end[2]-g['z']) > ztol for m in run):
            return skip('Variación Z dentro del contorno; no se completa una rampa.')
        if not _same_path(base, _primitives(run, tol), tol):
            return skip('Las capas cambian de geometría, dirección u orden; cierre no confirmado.')
    for g in refs:
        entry = motions[g['entry']]
        if _distance(entry.start, end) > tol or _distance(entry.end, start) > tol:
            return skip('El regreso no coincide con el mismo seam en las capas de referencia.')
        if _distance(entry.start, entry.end) <= tol:
            return skip('No hay un regreso XY depositado que confirme la geometría faltante.')
    # Every reference contains this exact return, although its original Z belongs
    # to the next level. It is the evidence used to close the first and last level.
    template = motions[refs[0]['entry']]
    report.update(review_required=True, status='DETECTED', reason='Regreso XY/Z repetido entre contornos concordantes.',
                  reference_layers=list(range(2, len(groups)+1)), affected_layer_count=len(groups),
                  missing_first_layer_length_mm=gap, seam_xy=list(start),
                  inferred_approach_source_line=first_move.line if approach is not None else None,
                  approach_motion_index=approach, first_motion_index=first, last_motion_index=last,
                  transition_motion_indices=[g['entry'] for g in refs],
                  closure_geometry={'kind': 'LINEAR', 'start_xy': list(end), 'end_xy': list(start),
                                    'source_line': template.line})
    for i, g in enumerate(groups):
        tail = motions[g['indices'][-1]]
        reference = motions[groups[i+1]['entry']] if i+1 < len(groups) else template
        report['layers'].append({'layer_index': i, 'source_z_mm': g['z'], 'seam_xy': list(start),
                                 'end_xy': list(end), 'closure_length_mm': gap,
                                 'source_line': reference.line if i+1 < len(groups) else tail.line,
                                 'reference_source_line': reference.line,
                                 'action': 'REASSIGN_RETURN_BEFORE_Z' if i+1 < len(groups) else 'ADD_FINAL_RETURN'})
    return report


def review_source(source, config):
    blocks = parse(source)
    motions, _ = interpret(blocks)
    review = analyze_closure(motions, blocks, config)
    from .cordon import analyze_cordon
    review['cordon'] = analyze_cordon(motions, blocks, config, review)
    return review


def prepare_layer_closure(motions, blocks, config):
    report = analyze_closure(motions, blocks, config)
    warnings = []
    if not report['review_required']: return motions, report, warnings
    if config.layer_completion.decision == 'preserve':
        report['status'] = 'PRESERVED'
        warnings.append(f"FIRST_LAYER_OPEN: primera capa con {report['missing_first_layer_length_mm']:.3f} mm de cierre pendiente; recorrido conservado sin reparación.")
        return motions, report, warnings
    transitions = set(report['transition_motion_indices'])
    output = []
    seam = tuple(report['seam_xy'])
    for i, m in enumerate(motions):
        if i == report['approach_motion_index']:
            output.append(replace(m, kind='RAPID', original_kind='RAPID', deposition=False,
                                  layer_closure_role='INFERRED_APPROACH'))
        elif i in transitions:
            closed = (*seam, m.start[2])
            output.append(replace(m, end=closed, layer_closure_role='REASSIGNED_RETURN',
                                  closure_reference_line=m.line))
            output.append(replace(m, start=closed, kind='Z_ONLY', deposition=False,
                                  layer_closure_role='RISE_AFTER_CLOSURE'))
        else:
            output.append(m)
        if i == report['last_motion_index']:
            output.append(replace(m, start=m.end, end=(*seam, m.end[2]), kind='LINEAR', original_kind='LINEAR', arc={},
                                  layer_closure_role='ADDED_FINAL_RETURN',
                                  closure_reference_line=report['closure_geometry']['source_line']))
    report.update(status='COMPLETED', reassigned_return_count=len(transitions), added_closure_count=1,
                  added_length_mm=report['missing_first_layer_length_mm'])
    return output, report, warnings


def validate_completed_closure(report, samples, config):
    """Check the printable plan after fillets/sampling and before export."""
    if report['status'] != 'COMPLETED': return []
    strokes = {}
    for s in samples:
        if s.deposition_active and s.stroke_id is not None and s.cordon_role != 'CORDON_PRIME':
            strokes.setdefault(s.stroke_id, []).append(s)
    rows, errors = [], []
    tol = config.geometry.max_xy_deviation_mm
    for members in strokes.values():
        a, b = members[0], members[-1]
        gap = math.dist((a.X, a.Y, a.Z), (b.X, b.Y, b.Z))
        rows.append({'layer_id': a.layer_id, 'stroke_id': a.stroke_id, 'gap_mm': gap,
                     'start_xyz': [a.X, a.Y, a.Z], 'end_xyz': [b.X, b.Y, b.Z]})
        if gap > tol: errors.append(f'FAIL_LAYER_CLOSURE: capa {a.layer_id} tiene {gap:.6f} mm de separación.')
    previous = None
    transition_count = 0
    max_xy = 0.
    for s in samples:
        if not s.emit: continue
        if s.generated_reason == 'LAYER_TRANSITION' and previous is not None:
            transition_count += 1
            max_xy = max(max_xy, math.hypot(s.X-previous.X, s.Y-previous.Y))
        previous = s
    if len(rows) != report['affected_layer_count'] or transition_count != len(rows)-1 or max_xy > tol:
        errors.append('FAIL_LAYER_CLOSURE: cantidad de contornos o subida vertical incorrecta.')
    report['validation'] = {'status': 'FAIL' if errors else 'PASS', 'layers': rows,
                            'max_gap_mm': max((r['gap_mm'] for r in rows), default=0.),
                            'transition_count': transition_count, 'max_transition_xy_mm': max_xy}
    return errors
