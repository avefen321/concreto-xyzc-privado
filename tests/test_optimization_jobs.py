import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from backend.optimization.jobs import JobManager,JobError,enabled
from backend.optimization.baseline import baseline_config
from tests.native_arc_benchmarks import path


class JobTests(unittest.TestCase):
    def test_cancel_keeps_a_validated_candidate(self):
        from backend.optimization.search import search as real_search
        def slow_search(objective,*args,**kwargs):
            def slow_objective(params):
                time.sleep(.04)
                return objective(params)
            return real_search(slow_objective,*args,**kwargs)
        with tempfile.TemporaryDirectory() as folder:
            manager=JobManager(Path(folder))
            with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}),patch('backend.optimization.jobs.search',slow_search):
                state=manager.start({'source':path([(0.,0.),(100.,0.)]),'config':baseline_config().to_dict(),
                                     'method':'random','budget':30,'seed':7})
                job=manager.get(state['id']);deadline=time.monotonic()+15
                while job.state.get('completed',0)<1 and time.monotonic()<deadline: time.sleep(.005)
                manager.cancel(job.id)
                while job.state['status'] in ('RUNNING','QUEUED') and time.monotonic()<deadline: time.sleep(.01)
                self.assertEqual(job.state['status'],'CANCELLED',job.public())
                self.assertTrue(job.state['best']['feasible'])
                self.assertTrue((job.directory/'selected.ngc').is_file())
                self.assertTrue((job.directory/'history.jsonl').is_file())
            manager.pool.shutdown(wait=True)
    def test_feature_flag_and_single_result_persistence(self):
        with tempfile.TemporaryDirectory() as folder:
            manager=JobManager(Path(folder))
            request={'source':path([(0.,0.),(100.,0.)]),'config':baseline_config().to_dict(),
                     'method':'random','budget':2,'seed':7}
            with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'0'}):
                with self.assertRaises(JobError): manager.start(request)
            with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
                state=manager.start(request); job=manager.get(state['id'])
                deadline=time.monotonic()+15
                while job.state['status'] in ('QUEUED','RUNNING') and time.monotonic()<deadline: time.sleep(.05)
                self.assertEqual(job.state['status'],'COMPLETED',job.public())
                self.assertTrue(job.state['best']['coverage_complete'])
                self.assertEqual(job.result.gcode,(job.directory/'selected.ngc').read_text(encoding='utf-8'))
                self.assertEqual(job.result.simulation['conversion_id'],job.result.final_motion_plan.conversion_id)
                self.assertTrue((job.directory/'history.csv').exists())
            manager.pool.shutdown(wait=True)

    def test_public_hosting_and_invalid_payload_remain_blocked(self):
        with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'true'}):
            self.assertFalse(enabled())
        with tempfile.TemporaryDirectory() as folder:
            manager=JobManager(Path(folder))
            with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
                with self.assertRaises(JobError): manager.start({'source':'X'*2_000_001})
                with self.assertRaises(JobError): manager.start({'source':'G1 X10 F600','config':baseline_config().to_dict(),'budget':True})
            manager.pool.shutdown(wait=True)
