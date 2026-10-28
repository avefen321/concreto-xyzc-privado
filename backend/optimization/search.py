"""Optional finite mixed-space BO/GP/EI, random and Optuna/TPE adapters."""
import importlib.util
import importlib
from functools import lru_cache
import itertools
import math
import random
import time
import warnings

SPACE={'candidate_window':(2,4,8),'split_level':(0,1),'max_sweep_deg':(90.,120.,160.)}
FIXED={'candidate_window':4,'split_level':0,'max_sweep_deg':160.}


@lru_cache(maxsize=1)
def dependency_diagnostics():
    """A discoverable package can still have a broken binary dependency."""
    diagnostics={}
    for method,modules in {'bo':('bayes_opt','sklearn.gaussian_process'), 'tpe':('optuna',)}.items():
        try:
            for name in modules: importlib.import_module(name)
            diagnostics[method]={'available':True,'reason':None}
        except Exception as error:
            diagnostics[method]={'available':False,'reason':f'{type(error).__name__}: {error}'}
    return diagnostics


def availability():
    return dict(random=True,**{k:v['available'] for k,v in dependency_diagnostics().items()})


def configurations(space=SPACE):
    return [dict(zip(space,values)) for values in itertools.product(*space.values())]


def search(objective,method='bo',budget=8,seed=7,*,space=SPACE,initial=4,cancel=None,progress=None,max_seconds=180.):
    if method not in ('bo','random','tpe'): raise ValueError('Método de búsqueda desconocido')
    if not availability()[method]: raise ValueError('Extra científico ausente: instale requirements-optimization-lock.txt')
    if type(budget) is not int or not 1<=budget<=60: raise ValueError('Presupuesto fuera de rango (1–60)')
    cancel=cancel or (lambda:False); progress=progress or (lambda state:None)
    rng=random.Random(seed); pool=configurations(space); names=list(space)
    initial=min(initial,budget); started=time.perf_counter()
    fixed={k:FIXED[k] for k in space} if all(FIXED.get(k) in values for k,values in space.items()) else pool[0]
    design=[dict(fixed)]; remaining=[p for p in pool if p!=fixed]; rng.shuffle(remaining)
    design.extend(remaining[:max(0,initial-1)])
    optimizer=study=None
    encode=lambda p:{k:space[k].index(p[k])/max(1,len(space[k])-1) for k in names}
    decode=lambda p:{k:space[k][max(0,min(len(space[k])-1,int(math.floor(p[k]*(len(space[k])-1)+.5))))] for k in names}
    if method=='bo':
        from bayes_opt import BayesianOptimization, acquisition
        from sklearn.gaussian_process.kernels import Matern
        optimizer=BayesianOptimization(f=None,pbounds={k:(0.,1.) for k in names},
                    acquisition_function=acquisition.ExpectedImprovement(xi=.01),random_state=seed,verbose=0)
        optimizer.set_gp_params(kernel=Matern(length_scale=[1.]*len(names),nu=2.5),
                                alpha=1e-8,normalize_y=True,n_restarts_optimizer=0)
    elif method=='tpe':
        import optuna
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        study=optuna.create_study(direction='minimize',sampler=optuna.samplers.TPESampler(seed=seed,n_startup_trials=initial))
        for p in design: study.enqueue_trial(p)
    rows=[]; best=None; seen=set(); reason='BUDGET'
    for index in range(budget):
        if cancel(): reason='CANCELLED'; break
        if time.perf_counter()-started>=max_seconds: reason='TIME_LIMIT'; break
        if len(seen)==len(pool): reason='SPACE_EXHAUSTED'; break
        proposed=time.perf_counter(); trial=None; mu=sigma=None
        novelty_fallback=False
        if method=='tpe':
            trial=study.ask(); params={k:trial.suggest_categorical(k,list(values)) for k,values in space.items()}
        elif index<len(design): params=design[index]
        elif method=='bo':
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                params=decode(optimizer.suggest())
            # The model is trained on canonical encoded configurations, never
            # on arbitrary floating values that round to a different action.
            key=tuple(params[k] for k in names)
            unused=[p for p in pool if tuple(p[k] for k in names) not in seen]
            if key in seen and unused:
                params=rng.choice(unused); novelty_fallback=True
        else: params=rng.choice(pool)
        key=tuple(params[k] for k in names); duplicate=key in seen; seen.add(key)
        selection_s=time.perf_counter()-proposed
        try:
            outcome=objective(params)
        except InterruptedError:
            reason='CANCELLED'; break
        value=float(outcome['objective'])
        if not math.isfinite(value): raise ValueError('El evaluador devolvió un costo no finito')
        if method=='bo':
            if len(optimizer.space) and hasattr(optimizer._gp,'X_train_'):
                try:
                    import numpy as np
                    prediction=optimizer._gp.predict(np.array([[encode(params)[k] for k in optimizer.space.keys]]),return_std=True)
                    mu=-float(np.asarray(prediction[0]).reshape(-1)[0])
                    sigma=float(np.asarray(prediction[1]).reshape(-1)[0])
                except (ValueError,AttributeError): pass
            if not duplicate:
                optimizer.register(params=encode(params),target=-value)
        elif method=='tpe': study.tell(trial,value)
        if outcome['feasible'] and (best is None or value<best['objective']):
            best={'parameters':dict(params),'objective':value,'iteration':index+1}
        row={'iteration':index+1,'method':method,'seed':seed,'parameters':dict(params),
             'objective':value,'feasible':bool(outcome['feasible']),'duplicate':duplicate,
             'acquisition':('INITIAL_DESIGN' if index<initial else 'EI_NOVELTY_FALLBACK' if novelty_fallback else 'EI') if method=='bo' else ('INITIAL_RANDOM' if index<initial else 'TPE_DENSITY_RATIO') if method=='tpe' else 'UNIFORM_RANDOM',
             'gp_mean_cost':mu,'gp_std_cost':sigma,'selection_s':selection_s,
             'elapsed_s':time.perf_counter()-started,'best_so_far':best['objective'] if best else None,
             'evaluations':outcome.get('evaluations',[]),'failure_reason':outcome.get('failure_reason')}
        rows.append(row); progress({'completed':len(rows),'budget':budget,'best':best,'last':row})
    return {'method':method,'seed':seed,'budget_requested':budget,'evaluations_attempted':len(rows),
            'initial_design':design,'best':best,'history':rows,'stop_reason':reason,
            'total_seconds':time.perf_counter()-started,'space':space,
            'unique_configurations':len(seen),'adaptive_evaluations':sum(r['acquisition'] in ('EI','EI_NOVELTY_FALLBACK','TPE_DENSITY_RATIO') for r in rows),
            'gp_kernel':'Matern_5_2' if method=='bo' else None,'physical_execution_verified':False}


def objective_for(evaluators,baselines):
    """No reserved geometry is passed here. Invalid trials count in the budget."""
    if not evaluators: raise ValueError('Faltan geometrías de calibración')
    def objective(params):
        rows=[]; feasible=True
        for evaluator,base in zip(evaluators,baselines):
            e=evaluator.evaluate(params).metrics
            rows.append(e)
            old=evaluator.evaluate(None).metrics
            feasible=feasible and e['feasible'] and e['N_blocks']<=old['N_blocks']
            feasible=feasible and e['M_microblocks'] is not None and e['M_microblocks']<=base['M_microblocks']
            feasible=feasible and e['arc_microblocks'] is not None and e['arc_microblocks']+e['M_microblocks']<=base['arc_microblocks']+base['M_microblocks']
        ratios=[e['N_blocks']/base['N_blocks'] for e,base in zip(rows,baselines) if e['N_blocks'] is not None and base['N_blocks']>0]
        penalty=1+sum(ev.evaluate(None).metrics['N_blocks']/b['N_blocks'] for ev,b in zip(evaluators,baselines))/len(evaluators)
        value=sum(ratios)/len(rows) if feasible and len(ratios)==len(rows) else penalty
        return {'objective':value,'feasible':feasible,'evaluations':rows,
                'failure_reason':None if feasible else 'CERTIFICATE_OR_BLOCK_OR_MICROBLOCK_CONSTRAINT'}
    return objective
