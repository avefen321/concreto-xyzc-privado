"""Ordered geometric fixtures. Run with python -B -m tests.benchmarks."""
import json
import math
from pathlib import Path
from backend.core.config import Config
from backend.core.conversion import convert
from .test_optimization import path, SQUARE


def fixtures():
    def polar(n=128,start=0,sweep=2*math.pi,rx=50,ry=50):
        return [(rx*math.cos(start+sweep*i/n),ry*math.sin(start+sweep*i/n)) for i in range(n+1)]
    yield 'horizontal',path([(0,0),(100,0)])
    yield 'vertical',path([(0,0),(0,100)])
    yield 'diagonal',path([(0,0),(100,100)])
    yield 'square',SQUARE
    yield 'rectangle',path([(0,0),(150,0),(150,80),(0,80)],close=True)
    yield 'triangle',path([(0,0),(150,0),(75,130)],close=True)
    yield 'acute_triangle',path([(0,0),(180,0),(20,20)],close=True)
    yield 'hexagon',path(polar(n=6))
    yield 'circle',path(polar())
    yield 'semicircle',path(polar(n=64,sweep=math.pi))
    yield 'ellipse',path(polar(rx=75,ry=30))
    yield 'sine',path([(i,12*math.sin(i/20)) for i in range(181)])
    yield 's_curve',path([(i,30*math.tanh((i-60)/25)) for i in range(121)])
    yield 'spiral',path([((20+i*.04)*math.cos(i*.025),(20+i*.04)*math.sin(i*.025)) for i in range(501)])
    yield 'line_curve_line',path([(0,0),(30,0)]+[(30+20*math.sin(i*math.pi/64),20-20*math.cos(i*math.pi/64)) for i in range(1,33)]+[(50,60)])
    yield 'near_corners',path([(0,0),(20,0),(20,15),(40,15)])
    yield 'irregular_points',path([(50*math.cos(a),50*math.sin(a)) for a in [0,.03,.08,.14,.22,.31,.43,.58,.77,1,1.3,1.57]])
    yield 'wrap_180',path([(0,0),(-20,.5),(-40,-.5),(-60,-1)])
    yield 'multi_turn',path(polar(n=400,sweep=6*math.pi))
    yield 'multilayer_closed',SQUARE+'G0 Z10\nM3\nG1 X100 Y0 F600\nG1 X100 Y100\nG1 X0 Y100\nG1 X0 Y0\nM5\n'
    yield 'seam_corner',SQUARE
    yield 'seam_curve',path(polar())
    yield 'open_stroke',path([(0,0),(60,0),(60,60)])
    yield 'tiny_segments',path([(0,0),(.002,0),(.003,0),(1,0),(1,1)])
    yield 'travel_z_print','G21 G90\nG0 Z30\nG0 X50 Y50\nG0 Z10\nM3\nG1 X100 Y50 F600\nM5\nG0 X150\nM3\nG1 X150 Y100\nM5\n'


def run():
    output=[]
    for name,source in fixtures():
        config=Config.from_dict({'layer_transition':{'safe_seam':True}}) if name in ('seam_corner','seam_curve','multilayer_closed') else Config()
        result=convert(source,config)
        output.append({'case':name,'status':result.report['status'],'validators':result.report['validators'],
                       'metrics':result.report['metrics'],'errors':result.report['errors'],
                       'warnings':result.report['warnings'],'conversion_id':result.report['conversion_id']})
    return output


if __name__=='__main__':
    rows=run()
    Path('ASTRA_BENCHMARK_RESULTS.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    for row in rows: print(row['case'],row['status'],row['errors'])
    raise SystemExit(any(row['status']=='FAIL' for row in rows))
