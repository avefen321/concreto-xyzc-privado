import importlib.util
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from backend.optimization.jobs import JobManager
from backend.optimization.baseline import baseline_config
from tests.native_arc_benchmarks import path


@unittest.skipUnless(importlib.util.find_spec('fastapi'),'FastAPI opcional')
class OptimizationApiTests(unittest.TestCase):
    def test_fastapi_job_result_and_download(self):
        from fastapi.testclient import TestClient
        from backend.main import app
        with tempfile.TemporaryDirectory() as folder:
            manager=JobManager(Path(folder))
            with patch('backend.main.optimization_jobs',manager),patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
                with TestClient(app) as client:
                    body={'source':path([(0.,0.),(100.,0.)]),'config':baseline_config().to_dict(),'method':'random','budget':2}
                    r=client.post('/api/optimization',json=body)
                    self.assertEqual(r.status_code,200,r.text); key=r.json()['id']
                    deadline=time.monotonic()+15
                    while time.monotonic()<deadline:
                        state=client.get('/api/optimization/'+key).json()
                        if state['status'] not in ('RUNNING','QUEUED'): break
                        time.sleep(.05)
                    self.assertEqual(state['status'],'COMPLETED',state)
                    result=client.get('/api/optimization/'+key+'/result')
                    self.assertEqual(result.status_code,200)
                    conversion=result.json()['id']
                    code=client.get('/api/conversion/'+conversion+'/gcode')
                    downloaded=client.get('/api/optimization/'+key+'/artifact/selected.ngc')
                    self.assertEqual(code.text,downloaded.text)
                    sim=client.get('/api/simulate/'+conversion).json()
                    self.assertEqual(sim['conversion_id'],conversion)
                    self.assertEqual(client.get('/js/optimization.js').status_code,200)
            manager.pool.shutdown(wait=True)

    def test_disabled_extra_cannot_start_job(self):
        from fastapi.testclient import TestClient
        from backend.main import app
        with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'0'}),TestClient(app) as client:
            self.assertEqual(client.post('/api/optimization',json={}).status_code,503)
            self.assertFalse(client.get('/api/optimization/capabilities').json()['enabled'])
