"""Reproducible native-curve fixtures and before/after machine metrics."""
import json
import math
import sys
from pathlib import Path
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse, interpret, arc_geometry


def path(points, z=20, feed=2400):
    return 'G21 G90 G17\nG0 X%.9f Y%.9f Z%.9f\nM3\n' % (*points[0], z) + ''.join(
        'G1 X%.9f Y%.9f F%d\n' % (*p, feed) for p in points[1:]) + 'M5\n'


def fixtures():
    yield 'fillet_R30', path([(150, 0), (300, 0), (300, 300)])
    yield 'square_R30', path([(150, 0), (300, 0), (300, 300), (0, 300), (0, 0), (150, 0)])
    for n in (360, 720, 2000):
        yield f'circle_{n}', path([(50*math.cos(2*math.pi*i/n), 50*math.sin(2*math.pi*i/n)) for i in range(n+1)])
    yield 'ellipse', path([(75*math.cos(2*math.pi*i/720), 30*math.sin(2*math.pi*i/720)) for i in range(721)])
    yield 'sine', path([(i, 50*math.sin(2*math.pi*i/300)) for i in range(601)])
    yield 's_curve', path([(i*.5, 30*math.tanh((i*.5-60)/25)) for i in range(241)])
    yield 'spiral', path([((20+i*.04)*math.cos(i*.025), (20+i*.04)*math.sin(i*.025)) for i in range(501)])
    yield 'source_G2', 'G21 G90 G17\nG0 X50 Y0 Z20\nG2 X0 Y-50 I-50 J0 F2400\n'
    yield 'source_G3', 'G21 G90 G17\nG0 X50 Y0 Z20\nG3 X0 Y50 I-50 J0 F2400\n'
    yield 'multilayer_square', path([(150,0),(300,0),(300,300),(0,300),(0,0),(150,0)]) + 'G1 Z38\nM3\nG1 X300 Y0\nG1 X300 Y300\nG1 X0 Y300\nG1 X0 Y0\nG1 X150 Y0\nM5\n'
    wall = Path('pared_senoidal_concreto_3d.gcode')
    if wall.exists(): yield 'user_wall', wall.read_text(encoding='utf-8-sig')


def metrics(result):
    motions, _ = interpret(parse(result.gcode)) if result.gcode else ([], [])
    counts = {name: sum(m.kind == kind for m in motions) for name, kind in
              [('G0','RAPID'),('G1','LINEAR'),('G2','ARC_CW'),('G3','ARC_CCW')]}
    lengths=[]; seconds=0.; unknown=0
    for m in motions:
        length = math.dist(m.start, m.end)
        if m.kind.startswith('ARC'):
            _, _, r, _, sweep=arc_geometry(m)
            length=math.hypot(r*sweep, m.end[2]-m.start[2])
        elif m.kind=='LINEAR' and length: lengths.append(length)
        if m.kind!='RAPID' and m.feed: seconds+=60*length/m.feed
        elif length: unknown+=1
    return dict(counts, total_motion_blocks=len(motions), file_size_bytes=len(result.gcode.encode('utf-8')),
                min_linear_block_mm=min(lengths, default=None), ideal_feed_time_s=seconds,
                measured_execution_time_s=None, unknown_duration_blocks=unknown,
                status=result.report['status'], errors=result.report['errors'],
                input_G1=result.report['metrics'].get('input_G1'),input_G2=result.report['metrics'].get('input_G2'),
                input_G3=result.report['metrics'].get('input_G3'),
                native_arcs=result.report.get('native_arcs'), max_required_omega_c=result.report['kinematics']['max_omega_c_deg_s'])


def run(output,export=False):
    rows={}
    export_dir=Path('resultados_arcos_nativos_2026-10-06')
    if export: export_dir.mkdir(exist_ok=True)
    for name, source in fixtures():
        result=convert(source)
        rows[name]=metrics(result)
        if export:
            (export_dir/(name+'_xyzc.ngc')).write_text(result.gcode,encoding='utf-8')
            (export_dir/(name+'_report.json')).write_text(json.dumps(result.report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        print(name, rows[name]['status'], rows[name]['total_motion_blocks'], flush=True)
    Path(output).write_text(json.dumps(rows, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    return rows


if __name__=='__main__': run(sys.argv[1] if len(sys.argv)>1 else 'NATIVE_ARC_BENCHMARK_RESULTS.json', '--export' in sys.argv)
