import math
import unittest
from tests.benchmarks import fixtures
from tests.test_optimization import path, SQUARE
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.numerics import max_abs


class BenchmarkTests(unittest.TestCase):
    def test_progressive_suite(self):
        for name,source in fixtures():
            with self.subTest(case=name):
                cfg=Config.from_dict({'layer_transition':{'safe_seam':True}}) if name.startswith('seam') or name=='multilayer_closed' else Config()
                result=convert(source,cfg)
                self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
                self.assertTrue(result.gcode)
                self.assertEqual(result.report['validators']['Serialization']['status'],'PASS')
                self.assertEqual(result.report['metrics']['nan_count'],0)
                if name in ('circle','ellipse','sine','s_curve','spiral','irregular_points','multi_turn'):
                    self.assertFalse(result.report['fillets'])
                    self.assertFalse(result.report['corners'])

    def test_midpoint_is_insufficient_for_cubic_error(self):
        # 3t²−2t³−t is zero at both endpoints and the midpoint, not inside.
        error,location=max_abs([0,-1,3,-2])
        self.assertAlmostEqual(error,math.sqrt(3)/18,places=12)
        self.assertNotAlmostEqual(location,.5)

    def test_sampling_uses_xyz_length_on_ramp(self):
        r=convert('G21 G90\nM3\nG1 X1 Z100 F600\n',Config.from_dict({'sampling':{'max_linear_step_mm':5}}))
        points=[s for s in r.samples if s.deposition_active]
        self.assertLessEqual(max(math.dist((a.X,a.Y,a.Z),(b.X,b.Y,b.Z)) for a,b in zip(points,points[1:])),5+1e-8)

    def test_source_end_stops_output(self):
        r=convert('G1 X10 F600\nM2\nM3\nG1 X999\n')
        self.assertNotIn('999',r.gcode)
        self.assertNotIn('M3',r.gcode)

    def test_minus_perpendicular_sign(self):
        r=convert('G1 X100 F600\n',Config.from_dict({'orientation':{
            'perpendicular_sign':-1,'initial_orientation_mode':'HOME_ALIGNED_TANGENT'}}))
        sample=next(s for s in r.samples if s.deposition_active)
        self.assertEqual(sample.C_deg,0)
        self.assertEqual(sample.C_deg-sample.c_reference_shift_deg,-90)

    def test_recommend_does_not_change_feed(self):
        source=path([(0,0),(100,0),(100,100)])
        r=convert(source,Config.from_dict({'process':{'feed_policy':'RECOMMEND'},'kinematics':{'max_c_velocity_deg_s':1}}))
        self.assertTrue(any(p['recommended_feed_mm_min']<600 for p in r.report['feed_plan']['primitives']))
        self.assertTrue(all(s.feed_mm_min==600 for s in r.samples if s.deposition_active))

    def test_seam_preserves_process_event(self):
        source=SQUARE.replace('G1 X100.000000000 Y100.000000000 F600','G4 P1\nG1 X100.000000000 Y100.000000000 F600')
        r=convert(source,Config.from_dict({'layer_transition':{'safe_seam':True},
                                           'serialization':{'motion_only':False}}))
        self.assertIn('G4 P1',r.gcode)
        self.assertFalse(any(s['reindexed'] for s in r.report['safe_seams']))


if __name__=='__main__': unittest.main()
