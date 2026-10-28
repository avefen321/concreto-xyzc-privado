"""Bounded local jobs; never sends commands to hardware or enables hosting."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass,field
import csv
import io
import json
import os
from pathlib import Path
import threading
import time
import uuid
import sys

from ..core.config import Config
from ..core.states import digest
from ..core.reference_snapshot import freeze_reference
from ..core.motion_timing import estimate_time
from ..core.nozzle_footprint import inspect_footprint
from .evaluator import Evaluator
from .search import search,objective_for,availability,dependency_diagnostics,FIXED,SPACE
from .statistics import reduction
from .resources import ResourceProbe

ROOT=Path(__file__).resolve().parents[2]


class JobError(ValueError):
    def __init__(self,message,status=422): super().__init__(message); self.status=status


def enabled():
    return (os.environ.get('XYZC_OPTIMIZATION_ENABLED','').lower() in ('1','true','yes') and
            os.environ.get('XYZC_PUBLIC_ACCESS','').lower() not in ('1','true','yes'))


@dataclass
class Job:
    id: str
    request: dict
    directory: Path
    state: dict=field(default_factory=dict)
    event: threading.Event=field(default_factory=threading.Event)
    result: object=None

    def public(self):
        # Full certificates remain in downloadable artifacts. Repeatedly
        # transferring thousands of region records makes polling look frozen.
        def compact(value):
            if isinstance(value,dict):
                return {k:compact(v) for k,v in value.items() if k not in ('regions','rejection_examples')}
            if isinstance(value,list): return [compact(v) for v in value]
            return value
        state=compact(self.state)
        state['candidates']=[dict(row,recommended=row['id']==state.get('best_candidate_id')) for row in state.get('candidates',[])]
        return dict(state,id=self.id)


class JobManager:
    def __init__(self,directory=None):
        self.directory=Path(directory) if directory else ROOT/'results/computational/jobs'
        self.jobs={}; self.lock=threading.Lock(); self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='xyzc-optimization')
        self.preview_lock=threading.Lock()

    def start(self,request):
        if not enabled(): raise JobError('Optimización local desactivada. Inicie run.py o defina XYZC_OPTIMIZATION_ENABLED=1 en modo local.',503)
        source=request.get('source')
        if not isinstance(source,str) or len(source.encode('utf-8'))>2_000_000 or len(source.splitlines())>5000:
            raise JobError('Entrada de optimización inválida o demasiado grande (máximo 5000 líneas y 2 MB)')
        config=Config.from_dict(request.get('config'))
        method=request.get('method','bo'); budget=request.get('budget',8); seed=request.get('seed',7)
        if method not in availability() or not availability()[method]:
            reason=dependency_diagnostics().get(method,{}).get('reason','Método desconocido')
            raise JobError(f'Método {method} no disponible en {sys.executable}: {reason}. Abra ABRIR_PROGRAMA.bat con el entorno .venv-web.',503)
        if type(budget) is not int or not 1<=budget<=30 or type(seed) is not int or not 0<=seed<=2147483647:
            raise JobError('Presupuesto o semilla inválidos')
        with self.lock:
            if any(job.state.get('status') in ('QUEUED','RUNNING') for job in self.jobs.values()):
                raise JobError('Ya existe una optimización activa; espere o cancélela',409)
            if len(self.jobs)>=10:
                completed=next((key for key,job in self.jobs.items() if job.state.get('status') not in ('QUEUED','RUNNING')),None)
                if completed: del self.jobs[completed]  # Disk artifacts remain preserved.
            key=uuid.uuid4().hex; directory=self.directory/key; directory.mkdir(parents=True,exist_ok=False)
            req={'source':source,'config':config.to_dict(),'method':method,'budget':budget,'seed':seed,
                 'limits':request.get('limits'),'timing_policy':request.get('timing_policy')}
            job=Job(key,req,directory,{'status':'QUEUED','completed':0,'budget':budget,'best':None,'error':None,'candidates':[]})
            self.jobs[key]=job
            self.pool.submit(self._run,job)
        return job.public()

    def get(self,key):
        if key not in self.jobs: raise JobError('Trabajo no encontrado en esta sesión; sus archivos locales se conservan',404)
        return self.jobs[key]

    def cancel(self,key):
        job=self.get(key); job.event.set(); return job.public()

    def candidate(self,key,candidate_id):
        job=self.get(key)
        if candidate_id=='best': candidate_id=job.state.get('best_candidate_id')
        row=next((row for row in job.state.get('candidates',[]) if row['id']==candidate_id),None)
        if row is None: raise JobError('Candidata no encontrada',404)
        if not row['valid']: raise JobError('La candidata no superó la validación; no tiene G-code descargable',422)
        return job,row

    def candidate_file(self,key,candidate_id):
        job,row=self.candidate(key,candidate_id)
        path=job.directory/('candidate_'+row['id']+'.ngc')
        if not path.is_file(): raise JobError('El archivo de la candidata aún no está disponible',409)
        name='mejor_validada' if candidate_id=='best' else {'baseline':'original','constructor':'constructor_fijo'}.get(row['id'],row['id'].replace('evaluation-','evaluacion_'))
        return path,('xyzc_'+job.request['method']+'_'+name+'.ngc')

    def candidate_result(self,key,candidate_id):
        job,row=self.candidate(key,candidate_id)
        if job.state['status'] in ('QUEUED','RUNNING'): raise JobError('Espere a que termine la búsqueda o cancélela antes de abrir una candidata',409)
        if not self.preview_lock.acquire(blocking=False): raise JobError('Ya se está preparando otra candidata para el visor',409)
        try:
            if row['id']==job.state.get('best_candidate_id'):
                result=job.result
            else:
                # Replay one candidate on demand, rather than retaining thirty
                # complete simulations. Match its persisted file byte-for-byte.
                snapshot,base=freeze_reference(job.request['source'],Config.from_dict(job.request['config']))
                if snapshot.hash!=job.state['reference_hash']: raise JobError('La referencia o el motor cambió; inicie una búsqueda nueva',409)
                evaluation=Evaluator(snapshot,base).evaluate(row['parameters'])
                if not evaluation.metrics['feasible']: raise JobError('No se pudo volver a validar la candidata',409)
                result=evaluation.result
                result.report['validators']['GlobalReference']=evaluation.metrics['validation']
            path,_=self.candidate_file(key,row['id'])
            if result is None or result.gcode.encode('utf-8')!=path.read_bytes():
                raise JobError('El plan reconstruido no coincide con el G-code guardado',409)
            result.report['optimization_selection']={'job_id':job.id,'candidate_id':row['id'],'label':row['label'],
                'recommended':row['id']==job.state.get('best_candidate_id'),'parameters':row['parameters'],
                'method':job.request['method'],'baseline_N':job.state['baseline']['N_blocks'],'N_blocks':row['N_blocks']}
            return result
        finally: self.preview_lock.release()

    def _record_candidate(self,job,candidate_id,label,evaluation,*,iteration=None,acquisition=None,eligible=True,reason=None,duplicate=False):
        m=evaluation.metrics
        if reason=='CERTIFICATE_OR_BLOCK_OR_MICROBLOCK_CONSTRAINT':
            reason='; '.join(m['validation'].get('errors',[])) if not m['feasible'] else 'Cumple geometría, pero empeora bloques o fragmentación frente al constructor.'
        row={'id':candidate_id,'label':label,'iteration':iteration,'acquisition':acquisition,
             'parameters':m.get('parameters'),'valid':bool(m['feasible']),'eligible':bool(eligible and m['feasible']),
             'duplicate':duplicate,
             'reason':reason,'N_blocks':m['N_blocks'],'M_microblocks':m['M_microblocks'],
             'arc_microblocks':m['arc_microblocks'],'xy_error_mm':m['xy_error_mm'],'C_error_deg':m['C_error_deg'],
             'coverage_complete':m['coverage_complete'],'gcode_hash':digest(evaluation.result.gcode) if m['feasible'] else None}
        if m['feasible']:
            (job.directory/('candidate_'+candidate_id+'.ngc')).write_text(evaluation.result.gcode,encoding='utf-8',newline='\n')
        self.write(job.directory/('candidate_'+candidate_id+'.json'),dict(row,validation=m['validation']))
        # Replace the list atomically for concurrent status requests.
        job.state['candidates']=job.state['candidates']+[row]

    @staticmethod
    def write(path,data):
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

    def _run(self,job):
        job.state.update(status='RUNNING',phase='REFERENCE')
        started=time.perf_counter(); ev=None; selected=None
        resources=ResourceProbe().start()
        cancelled=lambda:job.event.is_set() or time.perf_counter()-started>180. or resources.peak_rss>800*1024**2
        try:
            self.write(job.directory/'request.json',job.request)
            (job.directory/'input.ngc').write_text(job.request['source'],encoding='utf-8',newline='\n')
            snapshot,base=freeze_reference(job.request['source'],Config.from_dict(job.request['config']),cancelled)
            self.write(job.directory/'reference.json',snapshot.to_dict())
            ev=Evaluator(snapshot,base,cancelled)
            a=ev.evaluate(None)
            if not a.metrics['feasible']: raise ValueError('Referencia sin validación integral: '+'; '.join(a.metrics['validation']['errors']))
            selected=a; job.result=a.result
            self._record_candidate(job,'baseline','Conversión original',a)
            job.state['best_candidate_id']='baseline'
            parametric=bool(json.loads(snapshot.spans_json))
            mode='PARAMETRIC_HERMITE' if parametric else 'FROZEN_MACHINE_PLAN'
            space=SPACE if parametric else {'candidate_window':(2,4,8,12,16)}
            job.state.update(reference_hash=snapshot.hash,reference_mode=mode,baseline=a.metrics,best=a.metrics,phase='CONSTRUCTOR')
            b=ev.evaluate(FIXED) if not cancelled() else a
            b_eligible=(b.metrics['feasible'] and b.metrics['N_blocks']<=a.metrics['N_blocks'] and
                        b.metrics['M_microblocks']+b.metrics['arc_microblocks']<=a.metrics['M_microblocks']+a.metrics['arc_microblocks'])
            if b is not a:
                self._record_candidate(job,'constructor','Constructor fijo',b,eligible=b_eligible,
                                       reason=None if b_eligible else 'No mejora la base o aumenta la fragmentación')
            if (b is not a and b.metrics['feasible'] and b.metrics['N_blocks']<=a.metrics['N_blocks'] and
                    b.metrics['M_microblocks']+b.metrics['arc_microblocks']<=a.metrics['M_microblocks']+a.metrics['arc_microblocks']):
                selected=b; job.result=b.result
                job.state['best_candidate_id']='constructor'
            else: b=a
            job.state.update(constructor=b.metrics,phase='SEARCH')
            objective=objective_for([ev],[b.metrics])
            def progress(info):
                nonlocal selected
                last=info['last']
                candidate=ev.evaluate(last['parameters'])
                candidate_id='evaluation-'+str(last['iteration'])
                self._record_candidate(job,candidate_id,'Evaluación '+str(last['iteration']),candidate,
                                       iteration=last['iteration'],acquisition=last['acquisition'],eligible=last['feasible'],
                                       reason=last.get('failure_reason'),duplicate=last.get('duplicate',False))
                if last['feasible'] and last['evaluations'][0]['N_blocks']<selected.metrics['N_blocks']:
                    if candidate.metrics['feasible'] and candidate.metrics['N_blocks']<selected.metrics['N_blocks']:
                        selected=candidate; job.result=candidate.result
                        job.state['best_candidate_id']=candidate_id
                job.state.update(completed=info['completed'],best=selected.metrics,last=info['last'])
                self.write(job.directory/'progress.json',job.public())
                with (job.directory/'history.jsonl').open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(info['last'],ensure_ascii=False,allow_nan=False)+'\n')
            run=search(objective,job.request['method'],job.request['budget'],job.request['seed'],space=space,
                       cancel=cancelled,progress=progress,max_seconds=max(.01,180-(time.perf_counter()-started)))
            self.write(job.directory/'search.json',run)
            history=io.StringIO(newline=''); writer=csv.writer(history)
            writer.writerow(['iteration','method','seed','objective','feasible','best_so_far','parameters'])
            for row in run['history']:
                writer.writerow([row[k] for k in ('iteration','method','seed','objective','feasible','best_so_far')]+[json.dumps(row['parameters'])])
            (job.directory/'history.csv').write_text(history.getvalue(),encoding='utf-8-sig',newline='')
            timing=estimate_time(job.result.final_motion_plan.primitives,job.request['limits'],
                                 policy=job.request['timing_policy'],gcode=job.result.gcode)
            summary={'reference_hash':snapshot.hash,'baseline':a.metrics,'constructor':b.metrics,'selected':selected.metrics,
                     'reduction_blocks_pct':reduction(a.metrics['N_blocks'],selected.metrics['N_blocks']),
                     'search_seconds':run['total_seconds'],'method':job.request['method'],'seed':job.request['seed'],
                     'stop_reason':run['stop_reason'],'history_count':len(run['history']),'timing':timing,
                     'reference_mode':mode,'unique_configurations':run['unique_configurations'],
                     'adaptive_evaluations':run['adaptive_evaluations'],
                     'best_candidate_id':job.state['best_candidate_id'],'parameters':selected.metrics.get('parameters'),
                     'outcome':'REDUCED' if selected.metrics['N_blocks']<a.metrics['N_blocks'] else 'NO_VALID_REDUCTION',
                     'note':('Se conserva la geometría y C de la conversión actual; solo se unen G1 colineales con C afín. Los arcos y eventos se conservan.'
                             if not parametric else 'Se comparan líneas, arcos y biarcos sobre la curva y C congelados.'),
                     'nominal_footprint':inspect_footprint(job.result.final_motion_plan.primitives,snapshot.config.nozzle),
                     'study_kind':'PER_FILE_OPTIMIZATION_NOT_RESERVED_GENERALIZATION'}
            job.result.report['optimization']=summary
            job.result.report['validators']['GlobalReference']=selected.metrics['validation']
            job.state.update(status='CANCELLED' if cancelled() else 'COMPLETED',best=selected.metrics,summary=summary,phase='DONE')
        except InterruptedError:
            job.state.update(status='CANCELLED',phase='DONE')
        except Exception as error:
            job.state.update(status='FAILED',error=str(error),phase='DONE')
        finally:
            if job.result is not None:
                if selected is not None:
                    job.state['best']=selected.metrics
                    job.result.report.setdefault('optimization',{'selected':selected.metrics,'stop_reason':job.state['status']})
                    job.result.report['validators']['GlobalReference']=selected.metrics['validation']
                (job.directory/'selected.ngc').write_text(job.result.gcode,encoding='utf-8',newline='\n')
                self.write(job.directory/'selected_report.json',job.result.report)
                self.write(job.directory/'selected_plan.json',[p.to_dict() for p in job.result.final_motion_plan.primitives])
            job.state['elapsed_s']=time.perf_counter()-started
            job.state['resources']=resources.stop()
            if time.perf_counter()-started>180.: job.state['stop_reason']='TIME_LIMIT'
            elif resources.peak_rss>800*1024**2: job.state['stop_reason']='MEMORY_LIMIT'
            self.write(job.directory/'job.json',job.public())


MANAGER=JobManager()


def capabilities():
    return {'enabled':enabled(),'algorithms':availability(),'max_budget':30,'max_seconds':180,
            'dependencies':dependency_diagnostics(),'python_executable':sys.executable,
            'integration_version':'2.2','pid':os.getpid(),
            'disabled_reason':None if enabled() else 'PUBLIC_ACCESS' if os.environ.get('XYZC_PUBLIC_ACCESS','').lower() in ('1','true','yes') else 'LOCAL_FLAG_OFF',
            'purpose':'BLOCK_REPRESENTATION','physical_control':False}
