import json
from dataclasses import FrozenInstanceError, replace
import unittest
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.reference_snapshot import freeze_reference, RepresentationSession
from backend.optimization.baseline import baseline_config
from backend.optimization.evaluator import Evaluator,validate_candidate
from tests.native_arc_benchmarks import fixtures
from tests.general_curve_fixtures import cases


class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=dict(cases())['organic_60_ccw']
        cls.snapshot,cls.base=freeze_reference(cls.source,baseline_config())

    def test_immutable_and_hash_changes_with_home(self):
        with self.assertRaises(FrozenInstanceError): self.snapshot.source='other'
        data=self.snapshot.config.to_dict(); data['orientation']['home_axis_deg']=90
        other,_=freeze_reference(self.source,Config.from_dict(data))
        self.assertNotEqual(self.snapshot.hash,other.hash)
        sess=RepresentationSession(self.snapshot,{'candidate_window':2,'split_level':0,'max_sweep_deg':160})
        with self.assertRaises(ValueError): convert(self.source,Config.from_dict(data),representation=sess)

    def test_dp_preserves_reference_and_reduces_problem_case(self):
        evaluator=Evaluator(self.snapshot,self.base)
        a=evaluator.evaluate(None); b=evaluator.evaluate({'candidate_window':4,'split_level':0,'max_sweep_deg':160})
        self.assertTrue(a.metrics['feasible'],a.metrics['validation'])
        self.assertTrue(b.metrics['feasible'],b.metrics['validation'])
        self.assertLess(b.metrics['N_blocks'],a.metrics['N_blocks'])
        self.assertEqual(a.metrics['reference_hash'],b.metrics['reference_hash'])
        self.assertTrue(b.metrics['coverage_complete'])
        self.assertLessEqual(b.metrics['xy_error_mm'],.30)
        self.assertLessEqual(b.metrics['C_error_deg'],1.)
        self.assertIsNone(b.metrics['t_print_real_s'])

    def test_export_tampering_and_missing_reference_do_not_pass(self):
        from copy import deepcopy
        result=deepcopy(self.base)
        result.gcode=result.gcode.replace('F2400.000','F9999.000',1)
        self.assertNotEqual(result.gcode,self.base.gcode)
        self.assertEqual(validate_candidate(self.snapshot,result)['status'],'FAIL')
        result=deepcopy(self.base); result.gcode=''
        self.assertFalse(validate_candidate(self.snapshot,result)['coverage_complete'])
        result=deepcopy(self.base)
        for p in result.final_motion_plan.primitives:
            if p.metadata.get('reference_id'):
                p.metadata['reference_id']='unknown'; break
        self.assertEqual(validate_candidate(self.snapshot,result)['status'],'FAIL')

    def test_native_and_no_movement_cases(self):
        source='G21 G90 G17\nG0 X50 Y0 Z20\nG3 X50 Y0 I-50 J0 F2400\n'
        snap,a=freeze_reference(source,baseline_config())
        self.assertTrue(Evaluator(snap,a).evaluate(None).metrics['feasible'])
        snap,a=freeze_reference('G21 G90\n',baseline_config())
        self.assertFalse(Evaluator(snap,a).evaluate(None).metrics['feasible'])

    def test_multilayer_180_degree_branch_is_frozen(self):
        import math
        from tests.native_arc_benchmarks import path
        points=[(float(i),20*math.sin(i/30)) for i in range(0,151,3)]
        source=path(points)+path(list(reversed(points)),z=38)
        snap,a=freeze_reference(source,baseline_config())
        e=Evaluator(snap,a).evaluate({'candidate_window':8,'split_level':0,'max_sweep_deg':160})
        self.assertTrue(e.metrics['feasible'],e.metrics['validation'])
        self.assertEqual(snap.orientation_json,RepresentationSession(snap).snapshot.orientation_json)


if __name__=='__main__': unittest.main()
