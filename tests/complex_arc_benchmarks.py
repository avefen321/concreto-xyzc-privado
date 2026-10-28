"""Compare 3.4.1 to saved 3.4.0 results, preserving historical artifacts."""
import json
from pathlib import Path
from backend.core.config import Config
from backend.core.conversion import convert
from tests.native_arc_benchmarks import fixtures,metrics
from tests.test_complex_arc_fitting import coarse_sine


def run():
    directory=Path('resultados_curvas_ij_2026-10-06'); directory.mkdir(exist_ok=True)
    baseline=json.loads(Path('NATIVE_ARC_BENCHMARK_RESULTS.json').read_text(encoding='utf-8'))
    results={}
    cases=[(name,source,Config()) for name,source in fixtures()]
    cases.extend((name,coarse_sine(),Config.from_dict({'sampling':{'max_linear_step_mm':1.},
                 'arc_fit':{'adaptive_refinement':refinement}}))
                 for name,refinement in (('coarse_sine_without_refinement',False),('coarse_sine_with_refinement',True)))
    for name,source,config in cases:
        result=convert(source,config); after=metrics(result)
        before=baseline.get(name)
        results[name]={'before_3_4_0':{k:before[k] for k in
                       ('status','total_motion_blocks','file_size_bytes','G1','G2','G3')} if before else None,
                       'after_3_4_1':after}
        (directory/(name+'_xyzc.ngc')).write_text(result.gcode,encoding='utf-8',newline='\n')
        (directory/(name+'_report.json')).write_text(json.dumps(result.report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        print(name,result.report['status'],'blocks',after['total_motion_blocks'],'arcs',after['G2']+after['G3'],
              'geometry',after['native_arcs']['max_fit_error_mm'],'C',after['native_arcs']['max_orientation_fit_error_deg'],flush=True)
        if result.report['status']=='FAIL': raise RuntimeError(result.report['errors'])
    Path('COMPLEX_CURVE_BENCHMARK_RESULTS.json').write_text(json.dumps(results,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    return results


if __name__=='__main__': run()
