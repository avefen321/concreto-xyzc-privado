import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from backend.optimization.jobs import JobManager,JobError
from backend.optimization.baseline import baseline_config


class CandidateSelectionTests(unittest.TestCase):
    def run_job(self,manager):
        source='G21 G90 G17\nG0 X0 Y0 Z20\nG1 X100 Y0 F2400\n'
        state=manager.start({'source':source,'config':baseline_config().to_dict(),'method':'random','budget':3})
        job=manager.get(state['id']);deadline=time.monotonic()+20
        while job.state['status'] in ('QUEUED','RUNNING') and time.monotonic()<deadline:time.sleep(.02)
        self.assertEqual(job.state['status'],'COMPLETED',job.public())
        return job

    def test_every_evaluation_is_listed_and_exact_gcode_can_be_replayed(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
            manager=JobManager(Path(folder))
            try:
                job=self.run_job(manager);rows=job.public()['candidates']
                self.assertEqual(len(rows),5)
                self.assertEqual(sum(row['recommended'] for row in rows),1)
                for row in rows:
                    self.assertTrue(row['valid'])
                    result=manager.candidate_result(job.id,row['id']);path,_=manager.candidate_file(job.id,row['id'])
                    self.assertEqual(result.gcode.encode('utf-8'),path.read_bytes())
                    self.assertEqual(result.simulation['conversion_id'],result.report['conversion_id'])
                    self.assertEqual(result.report['optimization_selection']['candidate_id'],row['id'])
                best=manager.candidate_result(job.id,'best')
                self.assertTrue(best.report['optimization_selection']['recommended'])
            finally:manager.pool.shutdown(wait=True)

    def test_invalid_candidates_and_traversal_never_download(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
            manager=JobManager(Path(folder))
            try:
                job=self.run_job(manager)
                job.state['candidates'].append({'id':'invalid','valid':False})
                with self.assertRaises(JobError):manager.candidate_file(job.id,'invalid')
                with self.assertRaises(JobError):manager.candidate_file(job.id,'../../request.json')
            finally:manager.pool.shutdown(wait=True)

    def test_fastapi_candidate_download_and_simulation_are_identical(self):
        from fastapi.testclient import TestClient
        from backend.main import app
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
            manager=JobManager(Path(folder))
            try:
                job=self.run_job(manager)
                with patch('backend.main.optimization_jobs',manager),TestClient(app) as client:
                    for row in job.public()['candidates']:
                        prefix=f'/api/optimization/{job.id}/candidate/{row["id"]}'
                        data=client.get(prefix+'/result');self.assertEqual(data.status_code,200,data.text)
                        result=data.json();code=client.get(prefix+'/gcode')
                        self.assertEqual(code.status_code,200)
                        self.assertEqual(code.content,client.get('/api/conversion/'+result['id']+'/gcode').content)
                        sim=client.get('/api/simulate/'+result['id']).json()
                        self.assertEqual(sim['audit']['optimization_selection']['candidate_id'],row['id'])
                    self.assertEqual(client.get(f'/api/optimization/{job.id}/candidate/best/gcode').status_code,200)
                    for path in ('/js/deposition.js','/js/viewer3d.js','/vendor/three/three.module.js','/vendor/three/three.core.js'):
                        response=client.get(path);self.assertEqual(response.status_code,200,path)
                        self.assertEqual(response.headers['cache-control'],'no-cache')
                    self.assertIn('/js/app.js?v=2.2',client.get('/').text)
                    self.assertEqual(client.get('/vendor/three/unknown.js').status_code,404)
            finally:manager.pool.shutdown(wait=True)

    def test_fallback_serves_candidates_and_vendor_without_fastapi(self):
        import threading
        import urllib.request
        from http.server import ThreadingHTTPServer
        from backend import fallback_server
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
            manager=JobManager(Path(folder));server=None
            try:
                job=self.run_job(manager)
                with patch('backend.fallback_server.optimization_jobs',manager):
                    server=ThreadingHTTPServer(('127.0.0.1',0),fallback_server.Handler)
                    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                    prefix=f'http://127.0.0.1:{server.server_port}/api/optimization/{job.id}/candidate/evaluation-2'
                    with urllib.request.urlopen(prefix+'/result') as response:data=json.load(response)
                    self.assertEqual(data['report']['optimization_selection']['candidate_id'],'evaluation-2')
                    with urllib.request.urlopen(prefix+'/gcode') as response:code=response.read()
                    with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/api/conversion/{data["id"]}/gcode') as response:self.assertEqual(code,response.read())
                    with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/vendor/three/three.core.js') as response:self.assertEqual(response.status,200)
                    server.shutdown();server.server_close();server=None;thread.join(timeout=3)
            finally:
                if server:server.shutdown();server.server_close()
                manager.pool.shutdown(wait=True)
