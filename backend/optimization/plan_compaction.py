"""Exact analytic joins on a frozen machine plan, including POLYLINE/G1 modes.

No curve is reconstructed and no tolerance is relaxed. Only collinear G1
pieces with the same affine C law can join. Native arcs stay analytic and
unchanged. Comments and machine commands form barriers in the exported file.
"""
from copy import deepcopy
from dataclasses import replace
import json
import math

from ..core.gcode import parse, interpret
from ..core.states import digest
from ..core.conversion import _serialize, _csv
from ..core.diagnostics import primitive_density
from ..core.primitives import analyze_primitives
from ..core.segmentation_optimizer import SearchParameters


EPS = 1e-9


def _same(a, b):
    return all(abs(x-y) <= EPS for x, y in zip(a, b))


def _line_chain(parts):
    """Analytic proof: every endpoint lies on one monotone XYZC segment."""
    first, last = parts[0], parts[-1]
    start, end = first.start_xyz, last.end_xyz
    direction = tuple(b-a for a, b in zip(start, end))
    norm2 = sum(d*d for d in direction)
    if norm2 <= EPS**2:
        return False
    cursor = 0.
    for p in parts:
        if p.type != 'LINE':
            return False
        for xyz, c in ((p.start_xyz, p.start_c), (p.end_xyz, p.end_c)):
            t = sum((x-a)*d for x, a, d in zip(xyz, start, direction))/norm2
            projected = tuple(a+t*d for a, d in zip(start, direction))
            if t < cursor-EPS or t > 1+EPS or not _same(xyz, projected):
                return False
            if abs(c-(first.start_c+t*(last.end_c-first.start_c))) > EPS:
                return False
            cursor = t
    return True


def _barriers(program):
    lines = {r['primitive_id']:r['output_line'] for r in program['mapping'] if r['primitive_id']}
    exact = {}; stopped = False
    for block in parse(program['gcode']):
        for letter, value in block.words:
            if letter == 'G' and value in (61, 61.1, 64):
                stopped = value != 64
        exact[block.line] = stopped
    return lines, exact


def compact_result(snapshot, baseline, parameters, cancel=None):
    params = SearchParameters.from_dict(parameters)
    config = snapshot.config
    cancel = cancel or (lambda: False)
    original = list(baseline.final_motion_plan.primitives)
    # The actual serialized order is authoritative. Joining cannot move a
    # dwell, feed-only command, M command, comment or exact-stop boundary.
    output_lines, exact = _barriers({'mapping':baseline.report['serialization_map'], 'gcode':baseline.gcode})
    samples = {s.sample_id: s for s in baseline.samples}

    def joinable(parts):
        first, last = parts[0], parts[-1]
        if any(p.type != 'LINE' or p.process_state != 'DEPOSIT' or p.E is not None for p in parts):
            return False
        if any((p.feed, p.layer_id, p.stroke_id, p.motion_type, p.generated_reason) !=
               (first.feed, first.layer_id, first.stroke_id, first.motion_type, first.generated_reason)
               for p in parts):
            return False
        if any(exact.get(output_lines[p.id], True) for p in parts):
            return False
        if any(output_lines[b.id] != output_lines[a.id]+1 for a, b in zip(parts, parts[1:])):
            return False
        if any(not _same(a.end_xyz, b.start_xyz) or abs(a.end_c-b.start_c)>EPS
               for a, b in zip(parts, parts[1:])):
            return False
        if any(samples[p.sample_id].is_corner or samples[p.sample_id].layer_closure_role or
               samples[p.sample_id].cordon_role for p in parts[:-1]):
            return False
        if abs(last.end_c-first.start_c) > config.sampling.max_delta_c_deg+EPS:
            return False
        if any(samples[p.sample_id].primitive_type != 'STRAIGHT' for p in parts):
            if math.dist(first.start_xyz, last.end_xyz) > config.sampling.max_linear_step_mm+EPS:
                return False
        return _line_chain(parts)

    n = len(original)
    costs = [0]+[math.inf]*n
    previous = [None]*(n+1)
    attempts = accepted = 0
    for end in range(1, n+1):
        if cancel():
            raise InterruptedError('Búsqueda cancelada')
        costs[end], previous[end] = costs[end-1]+1, end-1
        for start in range(max(0, end-params.candidate_window), end-1):
            attempts += 1
            if joinable(original[start:end]):
                accepted += 1
                if costs[start]+1 < costs[end]:
                    costs[end], previous[end] = costs[start]+1, start
    groups = []
    end = n
    while end:
        start = previous[end]
        groups.append(original[start:end]); end = start
    groups.reverse()
    stats = {'regions': 0, 'edges_attempted': attempts, 'edges_accepted': accepted,
             'fallback_regions': 0, 'reference_mode': 'FROZEN_MACHINE_PLAN',
             'merged_blocks': n-len(groups), 'active_parameters': ['candidate_window']}
    if len(groups) == n:
        return baseline, stats
    result = deepcopy(baseline)
    kept = []
    ids = set()
    sample_primitives = {}
    for group in groups:
        if len(group) == 1:
            p = group[0]
        else:
            first, last = group[0], group[-1]
            # Emit at the first source anchor, using the last endpoint/sample.
            p = replace(last, start_xyz=first.start_xyz, start_c=first.start_c,
                        anchor_line=first.anchor_line or first.source_line,
                        metadata=dict(last.metadata, compacted_primitive_ids=[q.id for q in group]))
        kept.append(p); ids.add(p.sample_id); sample_primitives[p.sample_id] = p
    for sample in result.samples:
        if sample.emit and sample.sample_id not in ids:
            sample.emit = False; sample.display_only = True
        if sample.sample_id in sample_primitives:
            sample.primitive_id = sample_primitives[sample.sample_id].id
    identity = digest([snapshot.hash, parameters, 'EXACT_PLAN_COMPACTION_V1'])
    result.final_motion_plan = replace(result.final_motion_plan, conversion_id=identity,
                                      samples=tuple(result.samples), primitives=tuple(kept))
    source_blocks = parse(snapshot.source)
    source_motions, _ = interpret(source_blocks)
    result.gcode, mapping = _serialize(source_blocks, source_motions, result.final_motion_plan, snapshot.config)
    result.report.update(conversion_id=identity, serialization_map=mapping)
    from ..core.validators import NativeArcValidator, SerializationValidator
    result.report['validators']['NativeArcs']=NativeArcValidator.check(result.gcode,kept,config)
    result.report['validators']['Serialization']=SerializationValidator.check(result.gcode,mapping,result.samples,config)
    result.report['feed_plan']['final_feeds']=[row for row in result.report['feed_plan']['final_feeds'] if row['sample_id'] in ids]
    result.simulation['process_plan']=result.report['feed_plan']['final_feeds']
    native, _ = analyze_primitives(kept, snapshot.config)
    result.report['native_arcs'].update(native)
    density = primitive_density(kept, result.report['metrics']['input_motion_count'], snapshot.config)
    result.report['density'] = density
    result.report['metrics'].update(output_motion_count=len(kept), output_G1_count=density['output_G1_count'],
                                    output_G2_count=density['output_G2_count'], output_G3_count=density['output_G3_count'],
                                    file_size_bytes=len(result.gcode.encode('utf-8')),
                                    native_compression_ratio=result.report['metrics']['input_motion_count']/max(1,len(kept)))
    # Stage names remain unchanged; this result has its own serialization hash.
    result.report['stages'][-1].update(output_hash=digest(result.gcode), count=len(result.gcode.splitlines()))
    result.csv_text = _csv(result.samples)
    sim = result.simulation
    sim.update(conversion_id=identity, samples=[s.to_dict() for s in result.samples],
               machine_primitives=[p.to_dict() for p in kept], audit=result.report)
    sim['final_motion_plan'].update(primitive_ids=[p.id for p in kept],
                                    emitted_sample_ids=[s.sample_id for s in result.samples if s.emit])
    return result, stats


def validate_plan_compaction(snapshot, result):
    """Independent full-coverage check; a metadata claim alone never passes."""
    from ..core.primitives import MotionPrimitive
    from dataclasses import fields
    baseline = json.loads(snapshot.baseline_primitives_json)
    program = json.loads(snapshot.machine_program_json)
    output_lines, exact = _barriers(program)
    source_group = {}; selected_group = {}
    cursor = 0
    for group_index,p in enumerate(result.final_motion_plan.primitives):
        members = p.metadata.get('compacted_primitive_ids', [p.id])
        originals = baseline[cursor:cursor+len(members)]
        if not members or [q['id'] for q in originals] != members:
            raise ValueError('Compactación: cobertura duplicada, omitida o fuera de orden')
        selected_group[p.id] = group_index
        for member in members: source_group[member] = group_index
        if len(members) == 1:
            if p.to_dict() != originals[0]:
                # JSON tuples/lists normalize before exact comparison.
                if json.loads(json.dumps(p.to_dict())) != originals[0]:
                    raise ValueError('Compactación: primitiva protegida modificada')
        else:
            cls = MotionPrimitive
            allowed = {f.name for f in fields(cls)}
            chain = [cls(**{k:v for k,v in q.items() if k in allowed}) for q in originals]
            if not _line_chain(chain):
                raise ValueError('Compactación: XYZ/C no es una misma recta afín')
            if p.type != 'LINE' or not _same(p.start_xyz, chain[0].start_xyz) or not _same(p.end_xyz, chain[-1].end_xyz):
                raise ValueError('Compactación: extremos o tipo modificados')
            if abs(p.start_c-chain[0].start_c)>EPS or abs(p.end_c-chain[-1].end_c)>EPS:
                raise ValueError('Compactación: rama angular modificada')
            if any((q.feed,q.layer_id,q.stroke_id,q.process_state,q.E) !=
                   (p.feed,p.layer_id,p.stroke_id,p.process_state,p.E) for q in chain):
                raise ValueError('Compactación: feed, capa o proceso modificados')
            if any(exact[output_lines[q.id]] for q in chain) or any(
                    output_lines[b.id] != output_lines[a.id]+1 for a,b in zip(chain,chain[1:])):
                raise ValueError('Compactación: unión cruza parada o comando protegido')
        cursor += len(members)
    if cursor != len(baseline):
        raise ValueError('Compactación: referencia incompleta')
    def events(gcode,mapping,groups):
        by_line={r['output_line']:r['primitive_id'] for r in mapping if r['primitive_id']}
        sequence=[]; seen=set()
        for block in parse(gcode):
            if block.line in by_line:
                group=groups[by_line[block.line]]
                if group not in seen: sequence.append(('MOTION',group));seen.add(group)
            elif block.words:
                sequence.append(('COMMAND',tuple(block.words)))
        return sequence
    if events(program['gcode'],program['mapping'],source_group) != events(
            result.gcode,result.report['serialization_map'],selected_group):
        raise ValueError('Compactación: orden relativo de proceso y movimientos modificado')
    return True
