"""Predeclared NEW holdout after repairing validation; no training or retuning."""
import argparse
import json
import math
from pathlib import Path
import statistics
from .benchmark import write,csv_file,save_result
from .evaluator import Evaluator
from .search import FIXED
from .statistics import reduction,paired_summary,wilson
from .resources import ResourceProbe
from ..core.config import Config
from ..core.reference_snapshot import freeze_reference
from ..core.states import digest


def fixtures():
    from tests.native_arc_benchmarks import path
    from scipy.special import fresnel
    # Entire families absent from the original calibration/holdout dataset.
    points=[]
    for i in range(97):
        t=2*math.pi*i/96
        signed=lambda x:math.copysign(math.sqrt(abs(x)),x)
        points.append((200+100*signed(math.cos(t)),180+65*signed(math.sin(t))))
    points[-1]=points[0]
    yield 'superellipse_n4','superellipse',path(points)
    ts=[.3+(2*math.pi-.6)*i/96 for i in range(97)]
    yield 'cycloid_open','cycloid',path([(50*(t-math.sin(t)),50*(1-math.cos(t))) for t in ts])
    points=[(200+100*math.sin(3*2*math.pi*i/192),180+80*math.sin(2*2*math.pi*i/192)) for i in range(193)]
    points[-1]=points[0]
    yield 'lissajous_3_2','lissajous',path(points)
    points=[]
    for i in range(97):
        s,c=fresnel(1.25*i/96);points.append((100*float(c),100*float(s)))
    yield 'euler_transition','euler_spiral',path(points)


def run(parent,output):
    parent=Path(parent);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    old=json.loads((parent/'run_manifest.json').read_text(encoding='utf-8'))
    winners=json.loads((parent/'frozen_selections.json').read_text(encoding='utf-8'))
    assert digest(winners)==old['selection_hash']
    data=list(fixtures());root=Path(__file__).resolve().parents[2]
    code_hash=digest({str(p.relative_to(root)):digest(p.read_text(encoding='utf-8'))
                      for folder in ('backend/core','backend/optimization') for p in sorted((root/folder).glob('*.py'))})
    manifest={'purpose':'FRESH_HOLDOUT_AFTER_CORRECTNESS_REPAIRS','selection_source':str(parent),
              'selection_hash':digest(winners),'config':old['config'],'code_hash':code_hash,
              'no_training':True,'no_post_result_retuning':True,'families_independent_for_pilot_only':True,
              'cases':[{'case':name,'family':family,'source_hash':digest(source),'split':'FRESH_RESERVED'} for name,family,source in data],
              'physical_data':None,'completed':False}
    # Declaration and inputs exist before the first conversion is evaluated.
    write(output/'run_manifest.json',manifest);write(output/'frozen_selections.json',winners)
    for name,family,source in data:
        (output/'inputs').mkdir(exist_ok=True);(output/'inputs'/(name+'.ngc')).write_text(source,encoding='utf-8',newline='\n')
    rows=[];family_rows=[];probe=ResourceProbe().start()
    for name,family,source in data:
        try:
            snapshot,a=freeze_reference(source,Config.from_dict(old['config'])); ev=Evaluator(snapshot,a)
        except ValueError as error:
            labels=['A','B']+[w['method']+'_'+str(w['seed']) for w in winners]
            for label in labels:
                rows.append({'case':name,'family':family,'split':'FRESH_RESERVED','method':label,
                             'status':'FAIL' if label=='A' else 'NOT_EVALUATED','feasible':False,
                             'coverage_complete':False,'N_blocks':None,'xy_error_mm':None,'C_error_deg':None,
                             'failure_reason':'BASELINE_CONVERSION_ERROR: '+str(error),'physical_data':None})
            family_rows.append(dict({'case':name,'family':family,'gain_A_B_pct':None,'gain_random_BO_pct':None},
                                    **{label:None for label in labels}))
            print(name,'BASELINE_CONVERSION_ERROR',str(error),flush=True)
            continue
        write(output/'references'/(name+'.json'),snapshot.to_dict())
        measurements={'A':ev.evaluate(None),'B':ev.evaluate(FIXED)}
        for winner in winners:
            measurements[winner['method']+'_'+str(winner['seed'])]=ev.evaluate(winner['parameters'])
        record={'case':name,'family':family}
        for method,e in measurements.items():
            row=dict(e.metrics,case=name,family=family,method=method,split='FRESH_RESERVED')
            rows.append(row)
            if e.result:save_result(output,name,method,e)
            record[method]=e.metrics['N_blocks'] if e.metrics['feasible'] else None
        record['gain_A_B_pct']=reduction(record['A'],record['B'])
        cs=[record['C_RANDOM_'+str(w['seed'])] for w in winners if w['method']=='C_RANDOM']
        ds=[record['D_BO_GP_EI_'+str(w['seed'])] for w in winners if w['method']=='D_BO_GP_EI']
        record['gain_random_BO_pct']=reduction(statistics.mean(cs),statistics.mean(ds)) if all(v is not None for v in cs+ds) else None
        family_rows.append(record)
        print(name,{key:value for key,value in record.items() if key in ('A','B','D_BO_GP_EI_7')},flush=True)
    summary={'n_families':len(data),'A_to_B':paired_summary([r['gain_A_B_pct'] for r in family_rows]),
             'random_to_BO':paired_summary([r['gain_random_BO_pct'] for r in family_rows]),
             'physical_data':None,'statistical_scope':'PREDECLARED_CONVENIENCE_PILOT_NOT_UNIVERSAL_ACCURACY'}
    success=sum(all(r['feasible'] for r in rows if r['family']==family) for _,family,_ in data)
    summary.update(successful_families_all_methods=success,wilson90_illustrative=wilson(success,len(data)))
    csv_file(output/'per_geometry.csv',rows);csv_file(output/'per_family.csv',family_rows)
    csv_file(output/'failures.csv',[r for r in rows if not r['feasible']]);write(output/'statistics_computational.json',summary)
    manifest.update(completed=True,resources=probe.stop());write(output/'run_manifest.json',manifest)
    (output/'REPORT_COMPUTATIONAL.md').write_text('# Verificación nueva predeclarada\n\n'+
        'Las cuatro familias se fijaron y guardaron antes de evaluarlas. Se conservaron las selecciones del piloto anterior; no hubo reentrenamiento ni ajuste de rangos.\n\n'+
        json.dumps(summary,ensure_ascii=False,indent=2)+'\n\n'+
        'No se interpreta el IC Wilson ilustrativo como exactitud universal. Con cuatro familias no se publica una prueba t automática ni una superioridad de IA no observada. Los fallos permanecen visibles.\n',encoding='utf-8')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--parent',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();run(args.parent,args.output)
