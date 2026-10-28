import math
from pathlib import Path
import unittest
from backend.core.reference_snapshot import freeze_reference
from backend.core.gcode import parse,interpret
from backend.optimization.baseline import baseline_config
from backend.optimization.evaluator import Evaluator
from backend.optimization.dense_control import dense_control
from tests.native_arc_benchmarks import path


class IntegrationStressTests(unittest.TestCase):
    def test_translated_curve_keeps_valid_counts(self):
        points=[(i*3.,15*math.sin(i/8)) for i in range(41)]
        counts=[]
        for dx,dy in ((0.,0.),(1000.,-500.)):
            snap,a=freeze_reference(path([(x+dx,y+dy) for x,y in points]),baseline_config())
            e=Evaluator(snap,a).evaluate({'candidate_window':4,'split_level':0,'max_sweep_deg':160})
            self.assertTrue(e.metrics['feasible'],e.metrics['validation'])
            counts.append(e.metrics['N_blocks'])
        self.assertEqual(counts[0],counts[1])

    def test_dense_control_has_no_arc_blocks_and_same_reference(self):
        source='G21 G90 G17\nG0 X50 Y0 Z20\nG3 X50 Y0 I-50 J0 F2400\n'
        snap,a=freeze_reference(source,baseline_config());e=dense_control(snap,a)
        self.assertTrue(e.metrics['feasible'],e.metrics)
        motions,_=interpret(parse(e.result.gcode))
        self.assertFalse(any(m.kind.startswith('ARC') for m in motions))
        self.assertEqual(e.result.final_motion_plan.conversion_id,e.result.simulation['conversion_id'])
        self.assertEqual(e.metrics['reference_hash'],snap.hash)
        self.assertLessEqual(e.metrics['xy_error_mm'],.3)
        self.assertLessEqual(e.metrics['C_error_deg'],1.)

    def test_incompatible_planes_and_unknown_motion_fail_explicitly(self):
        for source in ('G18 G2 X20 Z10 I10 K0 F600\n','G28\nG1 X10 F600\n','G5 X20 Y10 F600\n'):
            with self.assertRaises(ValueError): freeze_reference(source,baseline_config())
