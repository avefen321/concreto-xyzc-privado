"""Reproducible local calibration -> frozen selection -> reserved evaluation."""
import argparse
import csv
import io
import json
import platform
from pathlib import Path
import statistics
import sys
import time
import importlib.metadata

from .baseline import dataset,baseline_config
from .evaluator import Evaluator
from .search import FIXED,SPACE,search,objective_for
from .statistics import reduction,paired_summary,wilson
from ..core.reference_snapshot import freeze_reference
from ..core.nozzle_footprint import inspect_footprint
from ..core.states import digest
from ..core.config import Config


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def csv_file(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys); writer.writeheader()
        for row in rows:
            compact=dict(row)
            if isinstance(compact.get('validation'),dict):
                validation=compact['validation']
                compact['validation']={k:v for k,v in validation.items() if k not in ('regions','native_check','serialization')}
                compact['validation']['region_count']=len(validation.get('regions',[]))
                compact['validation']['detail']='See matching *_validation.json or search history JSON; CSV contains summary only'
            writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list,tuple)) else v for k,v in compact.items()})


def run(directory,budget=8,seeds=(7,19)):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter(); config=baseline_config()
    specs=list(dataset())
    manifest={'schema':1,'purpose':'COMPUTATIONAL_PILOT_FIXED_REFERENCE','budget':budget,'initial':min(4,budget),
              'seeds':seeds,'config':config.to_dict(),'space_proposed':SPACE,
              'dataset':[{'case':name,'split':split,'family':family,'source_hash':digest(source)} for name,split,family,source in specs],
              'versions':{p:importlib.metadata.version(p) for p in ('numpy','scipy','scikit-learn','bayesian-optimization','optuna','matplotlib')},
              'python':sys.version,'platform':platform.platform(),'reserved_selection_rule':'NO_RETUNING_AFTER_FROZEN_SELECTION',
              'physical_data':None,'statistical_population_claim':False,
              'hypothesis':'Exploratory: BO gain vs equal-budget random may be positive, zero or negative',
              'constraints':['GLOBAL_VALIDATION','N_NOT_GREATER_THAN_A','LINEAR_MICROBLOCKS_NOT_GREATER_THAN_B','ALL_MICROBLOCKS_NOT_GREATER_THAN_B']}
    write(directory/'run_manifest.json',manifest)
    csv_file(directory/'dataset_manifest.csv',manifest['dataset'])
    prepared={}; fixed=[]; rows=[]; candidate_rows=[]; histories=[]
    def prepare(spec):
        name,split,family,source=spec
        snapshot,a=freeze_reference(source,config); ev=Evaluator(snapshot,a)
        aa=ev.evaluate(None); bb=ev.evaluate(FIXED)
        print(name,'A',aa.metrics['N_blocks'],aa.metrics['feasible'],'B',bb.metrics['N_blocks'],bb.metrics['feasible'],flush=True)
        prepared[name]=(ev,aa,bb)
        write(directory/'references'/f'{name}.json',snapshot.to_dict())
        (directory/'inputs').mkdir(exist_ok=True)
        (directory/'inputs'/f'{name}.ngc').write_text(source,encoding='utf-8',newline='\n')
        for method,e in (('A',aa),('B',bb)):
            record=dict(e.metrics,case=name,split=split,family=family,method=method,seed=None)
            rows.append(record)
            if e.result is not None:
                save_result(directory,name,method,e)
        fixed.append({'case':name,'family':family,'split':split,'N_A':aa.metrics['N_blocks'],'N_B':bb.metrics['N_blocks'],
                      'gain_pct':reduction(aa.metrics['N_blocks'],bb.metrics['N_blocks']),
                      'A_valid':aa.metrics['feasible'],'B_valid':bb.metrics['feasible']})
        return ev,bb
    calibration=[spec for spec in specs if spec[1]=='calibration']
    for spec in calibration: prepare(spec)
    evaluators=[prepared[s[0]][0] for s in calibration]
    baselines=[prepared[s[0]][2].metrics for s in calibration]
    if not all(b['feasible'] and b['N_blocks']>0 for b in baselines):
        raise ValueError('El constructor fijo no pasa el gate; se conserva el diagnóstico, no se entrena IA')
    # One-factor sensitivity on CALIBRATION only, before touching reserved cases.
    sensitivity=[]; active={}
    for key,values in SPACE.items():
        signatures=[]
        for value in values:
            params=dict(FIXED,**{key:value})
            measured=[ev.evaluate(params).metrics for ev in evaluators]
            signature=[(m['N_blocks'],m['M_microblocks'],m['arc_microblocks'],m['feasible']) for m in measured]
            signatures.append(signature)
            sensitivity.append({'parameter':key,'value':value,'signature':signature})
        if any(s!=signatures[0] for s in signatures[1:]): active[key]=values
    if not active:
        write(directory/'sensitivity.json',sensitivity)
        raise ValueError('Ningún parámetro cambia el constructor; no se fabrica una contribución de IA')
    manifest['space_active']=active; manifest['dropped_parameters']=[k for k in SPACE if k not in active]
    write(directory/'sensitivity.json',sensitivity); write(directory/'run_manifest.json',manifest)
    objective=objective_for(evaluators,baselines)
    winners=[]
    for seed in seeds:
        for method,label in (('random','C_RANDOM'),('bo','D_BO_GP_EI'),('tpe','E_TPE')):
            print('SEARCH',method,seed,'space',active,flush=True)
            def wrapped(params): return objective(dict(FIXED,**params))
            search_result=search(wrapped,method,budget,seed,space=active,initial=min(4,budget),max_seconds=300.)
            histories.append(search_result)
            best=dict(FIXED,**(search_result['best']['parameters'] if search_result['best'] else {}))
            winners.append({'method':label,'algorithm':method,'seed':seed,'parameters':best,'seconds':search_result['total_seconds']})
            write(directory/'searches'/f'{label}_{seed}.json',search_result)
            for trial in search_result['history']:
                for spec,m in zip(calibration,trial['evaluations']):
                    candidate_rows.append(dict(m,case=spec[0],family=spec[2],split='calibration',
                                               method=label,seed=seed,iteration=trial['iteration'],
                                               objective=trial['objective'],trial_feasible=trial['feasible']))
    # This file is written BEFORE the first reserved conversion. No range or
    # winner changes are allowed below this line, even if BO performs poorly.
    write(directory/'frozen_selections.json',winners)
    manifest['selection_hash']=digest(winners); manifest['reserved_evaluation_started']=True
    write(directory/'run_manifest.json',manifest)
    for spec in specs:
        if spec[0] not in prepared: prepare(spec)
        ev,a,b=prepared[spec[0]]
        if spec[1]=='calibration': continue
        for winner in winners:
            e=ev.evaluate(winner['parameters'])
            row=dict(e.metrics,case=spec[0],family=spec[2],split=spec[1],method=winner['method'],seed=winner['seed'],
                     reduction_vs_A_pct=reduction(a.metrics['N_blocks'],e.metrics['N_blocks']))
            rows.append(row)
            if e.result is not None: save_result(directory,spec[0],winner['method']+'_'+str(winner['seed']),e)
    family_rows=[]
    for spec in specs:
        if spec[1]!='reserved': continue
        ev,a,b=prepared[spec[0]]
        record={'family':spec[2],'case':spec[0],'N_A':a.metrics['N_blocks'],'N_B':b.metrics['N_blocks']}
        for label in ('C_RANDOM','D_BO_GP_EI','E_TPE'):
            values=[r['N_blocks'] for r in rows if r['case']==spec[0] and r['method']==label and r['feasible']]
            record[label]=statistics.mean(values) if values else None
        record['gain_A_B_pct']=reduction(record['N_A'],record['N_B'])
        record['gain_random_BO_pct']=reduction(record['C_RANDOM'],record['D_BO_GP_EI'])
        family_rows.append(record)
    summary={'geometry_gain':paired_summary([r['gain_A_B_pct'] for r in family_rows]),
             'AI_gain_over_random':paired_summary([r['gain_random_BO_pct'] for r in family_rows]),
             'n_reserved_families':len(family_rows),'physical_times':None,
             'wilson_scope':'ILLUSTRATIVE_CONVENIENCE_PILOT_NOT_POPULATION_CERTIFICATION'}
    methods=[]
    for label in ('A','B','C_RANDOM','D_BO_GP_EI','E_TPE'):
        rr=[r for r in rows if r['split']=='reserved' and r['method']==label]
        family_successes=sum(all(r['feasible'] for r in rr if r['family']==f['family']) for f in family_rows)
        methods.append({'method':label,'reserved_records':len(rr),'reserved_families':len(family_rows),
                        'successful_families':family_successes,'wilson90_illustrative':wilson(family_successes,len(family_rows)),
                        'mean_N_records':statistics.mean(r['N_blocks'] for r in rr if r['N_blocks'] is not None) if rr else None})
    manifest['elapsed_s']=time.perf_counter()-started; manifest['completed']=True
    write(directory/'run_manifest.json',manifest); write(directory/'statistics_computational.json',summary)
    csv_file(directory/'per_candidate.csv',candidate_rows); csv_file(directory/'per_geometry.csv',rows)
    csv_file(directory/'per_family.csv',family_rows); csv_file(directory/'summary_methods.csv',methods)
    csv_file(directory/'failures.csv',[r for r in rows+candidate_rows if not r['feasible']])
    csv_file(directory/'ablation_geometry_A_B.csv',fixed)
    figures(directory,rows,family_rows,histories,methods)
    report(directory,manifest,fixed,summary,methods)
    return manifest


def save_result(directory,name,method,e):
    folder=directory/'exported_gcode'/name; folder.mkdir(parents=True,exist_ok=True)
    (folder/f'{method}.ngc').write_text(e.result.gcode,encoding='utf-8',newline='\n')
    write(folder/f'{method}_validation.json',e.metrics)
    write(folder/f'{method}_plan.json',[p.to_dict() for p in e.result.final_motion_plan.primitives])
    if method=='B':
        profile=Config.from_dict(e.result.report['config']).nozzle
        write(folder/'nominal_footprint.json',inspect_footprint(e.result.final_motion_plan.primitives,profile))


def figures(directory,rows,families,histories,methods):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=directory/'figures';folder.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'figure.dpi':140})
    fig,ax=plt.subplots(figsize=(8,4))
    for label,offset in (('N_A',-.18),('N_B',0),('D_BO_GP_EI',.18)):
        ax.bar([i+offset for i in range(len(families))],[(f[label] or 0)/f['N_A'] for f in families],width=.18,label=label)
    ax.set_xticks(range(len(families)),[f['family'] for f in families]);ax.set_ylabel('Bloques respecto al motor A');ax.legend();fig.tight_layout();fig.savefig(folder/'compression_by_family.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4))
    for run in histories:
        ax.plot([r['iteration'] for r in run['history']],[r['best_so_far'] for r in run['history']],label=f"{run['method']} · {run['seed']}")
    ax.set_xlabel('Evaluaciones de configuración');ax.set_ylabel('Mejor J válido en calibración');ax.legend();fig.tight_layout();fig.savefig(folder/'best_so_far_bo_vs_random.png');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(9,4))
    for axis,key,limit,unit in ((axes[0],'xy_error_mm',.3,'mm'),(axes[1],'C_error_deg',1.,'°')):
        data=[r[key] for r in rows if r['feasible'] and r[key] is not None]
        axis.scatter(range(len(data)),data,s=14);axis.axhline(limit,color='red',linestyle='--');axis.set_ylabel('Cota '+unit);axis.set_xlabel('Registro de programa validado')
    fig.tight_layout();fig.savefig(folder/'xy_c_errors.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4));ax.bar(range(len(histories)),[r['total_seconds'] for r in histories]);ax.set_xticks(range(len(histories)),[f"{r['method']} {r['seed']}" for r in histories]);ax.set_ylabel('Tiempo total de búsqueda (s)');fig.tight_layout();fig.savefig(folder/'computational_cost.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4));ax.bar([m['method'] for m in methods],[100*m['successful_families']/max(1,m['reserved_families']) for m in methods]);ax.set_ylim(0,105);ax.set_ylabel('Familias reservadas conformes (%)');fig.tight_layout();fig.savefig(folder/'constraint_compliance.png');plt.close(fig)


def report(directory,manifest,fixed,summary,methods):
    lines=['# Reporte computacional XYZC — piloto local','',
           'Se compararon referencias fijas; no se ejecutó CNC ni concreto. Los tiempos físicos permanecen null.',
           f"Presupuesto: {manifest['budget']} configuraciones por método y semilla; semillas {manifest['seeds']}. Las semillas no son nuevas geometrías.",'',
           '| Caso | Grupo | N A | N B | Reducción A→B % | B válido |','|---|---|---:|---:|---:|---|']
    for r in fixed: lines.append(f"| {r['case']} | {r['split']} | {r['N_A']} | {r['N_B']} | {r['gain_pct']:.2f} | {r['B_valid']} |")
    lines+=['',f"Ganancia geométrica por familia reservada: {summary['geometry_gain']}",'',f"Ganancia BO frente a aleatorio: {summary['AI_gain_over_random']}",'',
            'Los intervalos Wilson se incluyen solo como ilustración del piloto de conveniencia. No acreditan una población universal ni exactitud física. No se calcula t de Student con tres familias reservadas sin supuestos acreditados.',
            'Los errores se expresan en mm y grados. Las familias reservadas no intervinieron en sensibilidad ni selección. Las invalidaciones permanecen en failures.csv. No se afirma superioridad de BO cuando empata o pierde.',
            'El control G1 denso y cualquier limitación de cobertura se documentan en ESTADO_FASES.md; no se equipara Config.legacy con la misma referencia.',
            'El espacio activo, los parámetros descartados y los ganadores congelados están en run_manifest.json, sensitivity.json y frozen_selections.json.']
    (directory/'REPORT_COMPUTATIONAL.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    parser.add_argument('--budget',type=int,default=8);parser.add_argument('--seeds',type=int,nargs='+',default=[7,19])
    args=parser.parse_args();run(args.output,args.budget,tuple(args.seeds))
