"""Correctness re-audit of already frozen selections. NEVER retrains on reserve."""
import argparse
import json
from pathlib import Path
import statistics
import time
from .benchmark import write,csv_file,save_result,figures,report
from .evaluator import Evaluator
from .dense_control import dense_control
from .search import FIXED
from .statistics import paired_summary,reduction,wilson
from ..core.config import Config
from ..core.reference_snapshot import freeze_reference
from ..core.states import digest


def run(parent,output):
    parent=Path(parent); output=Path(output); output.mkdir(parents=True,exist_ok=False)
    old=json.loads((parent/'run_manifest.json').read_text(encoding='utf-8'))
    winners=json.loads((parent/'frozen_selections.json').read_text(encoding='utf-8'))
    assert digest(winners)==old['selection_hash']
    manifest=dict(old,purpose='CORRECTNESS_REAUDIT_OF_FROZEN_SELECTIONS',parent_run=str(parent),
                  reserved_reused_after_validator_fix=True,independent_fresh_holdout=False,
                  retraining_performed=False,completed=False)
    write(output/'run_manifest.json',manifest);write(output/'frozen_selections.json',winners)
    rows=[]; fixed=[]; dense_rows=[]; started=time.perf_counter()
    for spec in old['dataset']:
        source=(parent/'inputs'/(spec['case']+'.ngc')).read_text(encoding='utf-8')
        snapshot,base=freeze_reference(source,Config.from_dict(old['config']))
        ev=Evaluator(snapshot,base)
        write(output/'references'/(spec['case']+'.json'),snapshot.to_dict())
        (output/'inputs').mkdir(exist_ok=True);(output/'inputs'/(spec['case']+'.ngc')).write_text(source,encoding='utf-8',newline='\n')
        aa=ev.evaluate(None);bb=ev.evaluate(FIXED)
        print(spec['case'],'A',aa.metrics['N_blocks'],aa.metrics['feasible'],'B',bb.metrics['N_blocks'],bb.metrics['feasible'],flush=True)
        for label,e in (('A',aa),('B',bb)):
            rows.append(dict(e.metrics,**spec,method=label,seed=None))
            save_result(output,spec['case'],label,e)
        fixed.append(dict(spec,N_A=aa.metrics['N_blocks'],N_B=bb.metrics['N_blocks'],
                          gain_pct=reduction(aa.metrics['N_blocks'],bb.metrics['N_blocks']),
                          A_valid=aa.metrics['feasible'],B_valid=bb.metrics['feasible']))
        dense=dense_control(snapshot,base)
        rows.append(dict(dense.metrics,**spec,method='G1_DENSE',seed=None))
        dense_rows.append(dict(spec,N_G1=dense.metrics.get('N_blocks'),valid=dense.metrics['feasible'],
                               failure_reason=dense.metrics.get('failure_reason')))
        if dense.result: save_result(output,spec['case'],'G1_DENSE',dense)
        if spec['split']=='calibration': continue
        for winner in winners:
            e=ev.evaluate(winner['parameters'])
            rows.append(dict(e.metrics,**spec,method=winner['method'],seed=winner['seed']))
            if e.result: save_result(output,spec['case'],winner['method']+'_'+str(winner['seed']),e)
    families=[]
    for spec in old['dataset']:
        if spec['split']!='reserved':continue
        row={'case':spec['case'],'family':spec['family']}
        for label in ('A','B','C_RANDOM','D_BO_GP_EI','E_TPE'):
            matches=[r['N_blocks'] for r in rows if r['case']==spec['case'] and r['method']==label and r['feasible']]
            row['N_'+label]=statistics.mean(matches) if matches else None
        row['gain_A_B_pct']=reduction(row['N_A'],row['N_B'])
        row['gain_random_BO_pct']=reduction(row['N_C_RANDOM'],row['N_D_BO_GP_EI'])
        families.append(row)
    summary={'geometry_gain':paired_summary([r['gain_A_B_pct'] for r in families]),
             'AI_gain_over_random':paired_summary([r['gain_random_BO_pct'] for r in families]),
             'inference_scope':'CORRECTED_AUDIT_OF_ALREADY_SEEN_GEOMETRIES','physical_times':None}
    methods=[]
    for label in ('A','B','G1_DENSE','C_RANDOM','D_BO_GP_EI','E_TPE'):
        rr=[r for r in rows if r['split']=='reserved' and r['method']==label]
        successes=sum(any(r['family']==f['family'] for r in rr) and all(r['feasible'] for r in rr if r['family']==f['family']) for f in families)
        methods.append({'method':label,'reserved_records':len(rr),'reserved_families':len(families),
                        'successful_families':successes,'wilson90_illustrative':wilson(successes,len(families))})
    csv_file(output/'per_geometry.csv',rows);csv_file(output/'per_family.csv',families)
    csv_file(output/'summary_methods.csv',methods);csv_file(output/'failures.csv',[r for r in rows if not r['feasible']])
    csv_file(output/'ablation_geometry_A_B.csv',fixed);csv_file(output/'control_G1_dense.csv',dense_rows)
    write(output/'statistics_computational.json',summary)
    histories=[json.loads(p.read_text(encoding='utf-8')) for p in sorted((parent/'searches').glob('*.json'))]
    compatible=[dict(f,N_A=f['N_A'],N_B=f['N_B'],D_BO_GP_EI=f['N_D_BO_GP_EI']) for f in families]
    figures(output,rows,compatible,histories,methods)
    report(output,manifest,fixed,summary,methods)
    with (output/'REPORT_COMPUTATIONAL.md').open('a',encoding='utf-8') as f:
        f.write('\nADVERTENCIA DE DISEÑO: estas familias ya se observaron al depurar el verificador. Esta revisión mantiene los parámetros congelados y NO constituye una nueva prueba reservada independiente. La primera ejecución, incluidos los fallos, se conserva íntegra en la carpeta padre. Para inferencia futura se requiere un conjunto nuevo predefinido.\n')
    manifest.update(completed=True,reaudit_seconds=time.perf_counter()-started)
    write(output/'run_manifest.json',manifest)
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--parent',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();run(args.parent,args.output)
