"""Regressions for the real shortcut/environment and formerly blocked modes."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import runpy
import tempfile
import time
import unittest
from unittest.mock import patch

from backend.core.config import Config
from backend.core.reference_snapshot import freeze_reference
from backend.optimization.evaluator import Evaluator, validate_candidate
from backend.optimization.jobs import JobManager, JobError, capabilities
from backend.optimization.search import dependency_diagnostics

ROOT=Path(__file__).resolve().parents[1]
LINE='G21 G90 G17\nG0 X0 Y0 Z20\n'+''.join(f'G1 X{i} Y0 F2400\n' for i in range(1,21))


class ActivationTests(unittest.TestCase):
    def test_direct_launcher_redirects_to_project_environment_before_starting_server(self):
        with patch('sys.executable','C:/other/python.exe'),patch('os.execv',side_effect=RuntimeError('REDIRECT')) as execute:
            with self.assertRaisesRegex(RuntimeError,'REDIRECT'):
                runpy.run_path(str(ROOT/'run.py'),run_name='__main__')
        expected=str(ROOT/'.venv-web/Scripts/python.exe')
        self.assertEqual(Path(execute.call_args.args[0]),Path(expected))
        self.assertEqual(execute.call_args.args[1][:2],[expected,str(ROOT/'run.py')])

    def test_import_failure_is_reported_even_when_a_package_exists(self):
        dependency_diagnostics.cache_clear()
        try:
            with patch('backend.optimization.search.importlib.import_module',side_effect=ImportError('broken binary')):
                info=dependency_diagnostics()
                self.assertFalse(info['bo']['available'])
                self.assertIn('broken binary',info['bo']['reason'])
        finally: dependency_diagnostics.cache_clear()

    def test_capabilities_identify_runtime_and_public_mode(self):
        with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'true'}):
            info=capabilities()
        self.assertFalse(info['enabled']);self.assertEqual(info['disabled_reason'],'PUBLIC_ACCESS')
        self.assertTrue(info['python_executable']);self.assertEqual(info['integration_version'],'2.2')


class GeneralConversionTests(unittest.TestCase):
    def test_interpreter_only_uses_declared_initial_pose(self):
        from backend.core.gcode import parse,interpret
        blocks=parse('G21 G90\nG0 X10 Y10\nG1 X20 F600\n')
        default,_=interpret(blocks);declared,_=interpret(blocks,initial_xyz=(0,0,20))
        self.assertEqual(default[-1].end[2],0)
        self.assertEqual(declared[-1].start[2],20)
        self.assertEqual(declared[-1].end[2],20)
        with self.assertRaises(ValueError):interpret(blocks,initial_xyz=(0,0,float('nan')))

    def test_xy_only_native_circle_reparse_uses_prepositioned_Z(self):
        from backend.optimization.baseline import baseline_config
        cfg=baseline_config().to_dict();cfg['initial_xy_approach']['enabled']=True
        cfg['layer_transition']['safe_seam']=False
        source='G21 G90 G17\nG0 X50 Y0 Z20\nG3 X50 Y0 I-50 J0 F2400\n'
        snapshot,base=freeze_reference(source,Config.from_dict(cfg))
        self.assertTrue(Evaluator(snapshot,base).evaluate(None).metrics['feasible'])
        self.assertEqual(snapshot.initial_xyz,[0,0,20])

    def test_polyline_preserve_native_disabled_and_g1_only_keep_exact_configuration(self):
        source='G21 G90 G17\nG0 X50 Y0 Z20\nG3 X0 Y50 I-50 J0 F2400\nG1 X0 Y100\n'
        for value in ({},{'geometry':{'mode':'PRESERVE_PATH'}},{'native_arcs':{'enabled':False}},
                      {'serialization':{'motion_mode':'G1_ONLY'}}):
            with self.subTest(config=value):
                cfg=Config.from_dict(value);snapshot,base=freeze_reference(source,cfg)
                candidate=Evaluator(snapshot,base).evaluate({'candidate_window':8})
                self.assertTrue(candidate.metrics['feasible'],candidate.metrics['validation'])
                self.assertEqual(candidate.result.report['config'],cfg.to_dict())
                self.assertLessEqual(candidate.metrics['N_blocks'],len(base.final_motion_plan.primitives))

    def test_exact_collinear_cleanup_reduces_authored_dense_lines_and_syncs_viewer(self):
        cfg=Config.legacy()
        cfg=replace(cfg,simplification=replace(cfg.simplification,enabled=False))
        snapshot,base=freeze_reference(LINE,cfg)
        candidate=Evaluator(snapshot,base).evaluate({'candidate_window':4})
        self.assertTrue(candidate.metrics['feasible'],candidate.metrics['validation'])
        self.assertLess(candidate.metrics['N_blocks'],len(base.final_motion_plan.primitives))
        self.assertEqual(candidate.result.simulation['conversion_id'],candidate.result.final_motion_plan.conversion_id)
        self.assertEqual(candidate.result.simulation['machine_primitives'],[p.to_dict() for p in candidate.result.final_motion_plan.primitives])
        self.assertNotEqual(candidate.result.report['conversion_id'],base.report['conversion_id'])
        tampered=deepcopy(candidate.result)
        merged=next(p for p in tampered.final_motion_plan.primitives if p.metadata.get('compacted_primitive_ids'))
        merged.metadata['compacted_primitive_ids'].pop()
        self.assertEqual(validate_candidate(snapshot,tampered)['status'],'FAIL')

    def test_commands_and_exact_stop_block_joins(self):
        for source in (LINE.replace('G1 X10 Y0 F2400','G4 P1\nG1 X10 Y0 F2400'),LINE.replace('G17','G17 G61')):
            cfg=Config.legacy()
            cfg=replace(cfg,simplification=replace(cfg.simplification,enabled=False),
                        serialization=replace(cfg.serialization,motion_only=False))
            snapshot,base=freeze_reference(source,cfg)
            candidate=Evaluator(snapshot,base).evaluate({'candidate_window':16})
            self.assertTrue(candidate.metrics['feasible'],candidate.metrics['validation'])
            if 'G61' in source:
                self.assertEqual(candidate.metrics['N_blocks'],len(base.final_motion_plan.primitives))
            else:
                self.assertIn('G4 P1',candidate.result.gcode)

    def test_euler_degenerate_candidate_falls_back_without_changing_source(self):
        from backend.optimization.fresh_holdout import fixtures
        from backend.optimization.baseline import baseline_config
        source=next(source for _,family,source in fixtures() if family=='euler_spiral')
        snapshot,base=freeze_reference(source,baseline_config())
        self.assertTrue(Evaluator(snapshot,base).evaluate(None).metrics['feasible'])
        self.assertEqual(snapshot.source,source)
        self.assertTrue(any('DEGENERATE_PARAMETRIC_CANDIDATE' in str(d.get('split_reason'))
                            for d in base.report['native_arcs']['decisions']))

    def test_bo_and_tpe_really_run_after_initial_design_in_default_mode(self):
        for method in ('bo','tpe'):
            with self.subTest(method=method),tempfile.TemporaryDirectory() as folder:
                manager=JobManager(Path(folder))
                try:
                    with patch.dict(os.environ,{'XYZC_OPTIMIZATION_ENABLED':'1','XYZC_PUBLIC_ACCESS':'false'}):
                        state=manager.start({'source':LINE,'config':Config().to_dict(),'method':method,'budget':8})
                        job=manager.get(state['id']);deadline=time.monotonic()+30
                        while job.state['status'] in ('QUEUED','RUNNING') and time.monotonic()<deadline:time.sleep(.02)
                        self.assertEqual(job.state['status'],'COMPLETED',job.public())
                        self.assertGreater(job.state['summary']['adaptive_evaluations'],0)
                        history=json.loads((job.directory/'search.json').read_text(encoding='utf-8'))['history']
                        if method=='bo':self.assertTrue(any(r['gp_std_cost'] is not None for r in history))
                        self.assertNotIn('regions',job.public()['best']['validation'])
                        self.assertIsNotNone(job.result)
                finally:manager.pool.shutdown(wait=True)

    def test_real_ui_repeated_layers_do_not_depend_on_accumulated_C(self):
        from backend.optimization.baseline import baseline_config
        cfg=baseline_config().to_dict()
        cfg['orientation'].update(mode='alignment',symmetry_period_deg=180)
        cfg['initial_xy_approach']['enabled']=True
        cfg['xy_boundary_offset']['enabled']=True
        source=(ROOT/'codigos g testeo/01_s_inflexiones.ngc').read_text(encoding='utf-8-sig')
        snapshot,base=freeze_reference(source,Config.from_dict(cfg))
        self.assertNotEqual(base.report['status'],'FAIL',base.report['errors'])
        self.assertLessEqual(base.report['metrics']['max_layer_xy_drift_mm'],1e-6)
        candidate=Evaluator(snapshot,base).evaluate({'candidate_window':8,'split_level':0,'max_sweep_deg':160})
        self.assertTrue(candidate.metrics['feasible'],candidate.metrics['validation'])
