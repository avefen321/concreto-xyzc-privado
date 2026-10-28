"""Reproducible comparison and exports for the user-supplied G-code files."""
from pathlib import Path
import json
import math
import time
from backend.core.config import Config
from backend.core.conversion import convert
from tests.test_global_curves_z import layers
from tests.test_optimization import path


def run():
    output=Path('resultados_curvas_z20_z18');output.mkdir(exist_ok=True)
    baseline=json.loads(Path('CORNER_SAMPLING_BASELINE.json').read_text(encoding='utf-8'))
    rows=[]
    for filename in ['cruz_CA.gcode','hexagono_suave_CA.gcode','pared_senoidal_concreto_3d.gcode','examples/tres_capas_cuadrado.ngc']:
        source=Path(filename).read_text(encoding='utf-8-sig');t=time.perf_counter();r=convert(source);elapsed=time.perf_counter()-t
        assert r.report['status']!='FAIL',(filename,r.report['errors'])
        assert r.gcode==convert(source).gcode
        name=Path(filename).stem
        (output/f'{name}_xyzc.ngc').write_text(r.gcode,encoding='utf-8')
        (output/f'{name}_samples.csv').write_text(r.csv_text,encoding='utf-8')
        (output/f'{name}_report.json').write_text(json.dumps(r.report,ensure_ascii=False,indent=2),encoding='utf-8')
        before=next(v for v in baseline if Path(v['file']).name==Path(filename).name)
        row={'file':filename,'before':before,'after':{'seconds':elapsed,'status':r.report['status'],
             'counts':r.report['counts'],'density':r.report['density'],'z_conversion':{k:v for k,v in r.report['z_conversion'].items() if k!='motions'},
             'sampling':r.report['validators']['Sampling'],'kinematics':r.report['kinematics'],'warnings':r.report['warnings']},
             'G1_reduction_percent':100*(1-r.report['density']['output_G1_count']/before['G1'])}
        rows.append(row)
        print(filename,row['after']['density']['output_G1_count'],round(row['G1_reduction_percent'],2),flush=True)
    square=layers([0,20,40,60,80,100]);Path('examples/cuadrado_300_6_capas.ngc').write_text(square,encoding='utf-8')
    comparison={}
    for label,cfg in [('legacy',Config.legacy()),('optimized',Config())]:
        t=time.perf_counter();r=convert(square,cfg)
        comparison[label]={'seconds':time.perf_counter()-t,'density':r.report['density'],'kinematics':r.report['kinematics'],
                           'fillets':r.report['fillets'],'status':r.report['status']}
        (output/f'cuadrado_300_6_capas_{label}.ngc').write_text(r.gcode,encoding='utf-8')
    curves=[]
    for n in (40,4000):
        source=path([(30*math.cos(i*math.pi/2/n),30*math.sin(i*math.pi/2/n)) for i in range(n+1)])
        r=convert(source)
        curves.append({'input_vertices':n+1,'density':r.report['density'],'sampling':r.report['validators']['Sampling']})
    data={'user_files':rows,'square_300_6_layers':comparison,'same_curve_different_density':curves}
    Path('CORNER_SAMPLING_BENCHMARK_RESULTS.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    return data


if __name__=='__main__':run()
