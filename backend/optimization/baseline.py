"""Freeze the current motor's outputs before optimizing block representations.

Run from the repository root: python -m backend.optimization.baseline --output DIR
This is a baseline recorder, not an AI search or a global curve certificate.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import time

from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import arc_geometry, interpret, parse
from backend.core.states import BACKEND_VERSION, digest

ROOT = Path(__file__).resolve().parents[2]


def baseline_config():
    return Config.from_dict({
        'nozzle': {'length_mm': 35.0, 'width_mm': 15.0},
        'arc_fit': {'reference_model': 'PARAMETRIC_HERMITE'},
        'serialization': {'xyz_decimals': 6, 'c_decimals': 6, 'ijk_decimals': 6},
    })


def exported_metrics(gcode, microblock_mm, *, initial_xyz=None):
    """Count interpreted machine moves, including modal motion and C-only blocks."""
    motions, warnings = interpret(parse(gcode),initial_xyz=initial_xyz)
    counts = dict.fromkeys(('G0', 'G1', 'G2', 'G3'), 0)
    linear_microblocks = arc_microblocks = c_only = 0
    for motion in motions:
        code = {'RAPID': 'G0', 'LINEAR': 'G1', 'ARC_CW': 'G2', 'ARC_CCW': 'G3'}.get(motion.kind)
        if motion.kind == 'C_ONLY':
            code = 'G0' if motion.block.value('G') == 0 else 'G1'
            c_only += 1
        counts[code] += 1
        if not motion.deposition or motion.kind in ('RAPID', 'C_ONLY'):
            continue
        length = math.dist(motion.start, motion.end)
        if motion.kind.startswith('ARC'):
            _, _, radius, _, sweep = arc_geometry(motion)
            length = math.hypot(radius * sweep, motion.end[2] - motion.start[2])
            arc_microblocks += 0 < length < microblock_mm
        else:
            linear_microblocks += 0 < length < microblock_mm
    return {'motion_blocks': len(motions), 'counts': counts,
            'linear_deposition_microblocks': linear_microblocks,
            'arc_deposition_microblocks': arc_microblocks,
            'c_only_blocks': c_only, 'microblock_threshold_mm': microblock_mm,
            'file_size_bytes': len(gcode.encode('utf-8')), 'parser_warnings': warnings}


def snapshot(result, source, config, seconds):
    report = result.report
    measured = exported_metrics(result.gcode, config.sampling.preferred_min_block_length_mm)
    primitives = [p.to_dict() for p in result.final_motion_plan.primitives]
    reconstruction = report['curve_reconstruction']
    applied = reconstruction['applied']
    return {
        'schema_version': 1, 'backend_version': BACKEND_VERSION,
        'source_hash': digest(source), 'config_hash': digest(config.to_dict()),
        'conversion_id': report['conversion_id'], 'config': config.to_dict(),
        'baseline_plan_hash': digest(primitives), 'export_hash': digest(result.gcode),
        'status': report['status'], 'errors': report['errors'],
        'input': exported_metrics(source, config.sampling.preferred_min_block_length_mm),
        'export': measured, 'plan_motion_blocks': len(primitives),
        'plan_export_count_match': len(primitives) == measured['motion_blocks'],
        'display_sample_count': len(result.samples), 'conversion_time_s': seconds,
        'reference_model_requested': config.arc_fit.reference_model,
        'orientation_reference_applied': reconstruction['orientation_reference'],
        'reference_stage_hashes': report['stages'],
        'certificate_scope': 'RECONSTRUCTED_SMOOTH_SEGMENTS' if applied else None,
        'max_geometry_certificate_mm': reconstruction['max_certified_geometry_error_mm'] if applied else None,
        'max_orientation_certificate_deg': reconstruction['max_certified_orientation_error_deg'] if applied else None,
        'validators': report['validators'], 'optimizer_run': False,
        'machine_execution_verified': False, 'measured_execution_time_s': None,
        'global_reference_extracted': False,
    }


def dataset():
    # Existing fixtures are reused without altering their geometry or source feed.
    from tests.general_curve_fixtures import cases
    from tests.native_arc_benchmarks import fixtures, path
    general = dict(cases())
    native = dict(fixtures())
    yield 'straight_100', 'control', 'straight', path([(i * 3., 0.) for i in range(101)])
    yield 'native_circle', 'control', 'circle', (
        'G21 G90 G17\nG0 X50 Y0 Z20\nM3\n'
        'G3 X-50 Y0 I-50 J0 F2400\nG3 X50 Y0 I50 J0\nM5\n')
    yield 'fillet_square', 'control', 'square', native['square_R30']
    yield 'ellipse', 'calibration', 'ellipse', native['ellipse']
    yield 'open_s', 'calibration', 's_curve', native['s_curve']
    yield 'organic_60', 'calibration', 'organic', general['organic_60_ccw']
    yield 'spiral', 'reserved', 'spiral', native['spiral']
    yield 'four_lobes', 'reserved', 'lobes', general['four_lobes_120']
    if 'user_wall' not in general:
        raise FileNotFoundError('Falta pared_senoidal_concreto_3d.gcode para el caso reservado multicapa')
    yield 'user_wall', 'reserved', 'multilayer_wall', general['user_wall']


def run(output):
    # Never silently replace a frozen baseline.
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    config = baseline_config()
    code_files = sorted((ROOT / 'backend' / 'core').glob('*.py'))
    manifest = {'schema_version': 1, 'purpose': 'BASELINE_ONLY',
                'code_hash': digest({p.name: digest(p.read_text(encoding='utf-8-sig')) for p in code_files}),
                'config': config.to_dict(), 'config_hash': digest(config.to_dict()),
                'cases': [], 'ai_installed_or_executed': False}
    failures = []
    for name, split, family, source in dataset():
        case_dir = output / name
        case_dir.mkdir()
        (case_dir / 'input.ngc').write_text(source, encoding='utf-8', newline='\n')
        started = time.perf_counter()
        result = convert(source, config)
        row = snapshot(result, source, config, time.perf_counter() - started)
        row.update(case=name, split=split, family=family)
        for filename, data in (('baseline.json', row), ('report.json', result.report),
                               ('machine_plan.json', [p.to_dict() for p in result.final_motion_plan.primitives])):
            (case_dir / filename).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        (case_dir / 'baseline.ngc').write_text(result.gcode, encoding='utf-8', newline='\n')
        manifest['cases'].append(row)
        if row['status'] == 'FAIL' or not row['plan_export_count_match']:
            failures.append(name)
        print(name, row['status'], row['input']['motion_blocks'], '->', row['export']['motion_blocks'], flush=True)
    manifest['failed_cases'] = failures
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    buffer = io.StringIO(newline='')
    writer = csv.writer(buffer)
    writer.writerow(['case', 'split', 'status', 'input_blocks', 'baseline_blocks', 'G1', 'G2', 'G3', 'micro_G1', 'micro_arc', 'conversion_s'])
    for row in manifest['cases']:
        m = row['export']
        writer.writerow([row['case'], row['split'], row['status'], row['input']['motion_blocks'], m['motion_blocks'],
                         m['counts']['G1'], m['counts']['G2'], m['counts']['G3'],
                         m['linear_deposition_microblocks'], m['arc_deposition_microblocks'], row['conversion_time_s']])
    (output / 'summary.csv').write_text(buffer.getvalue(), encoding='utf-8-sig', newline='')
    if failures:
        raise RuntimeError('Línea base con fallos: ' + ', '.join(failures))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    run(args.output)
