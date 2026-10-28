"""Check the physical profile and the exported-block baseline contract."""
import unittest

from backend.core.config import Config
from backend.core.conversion import convert
from backend.optimization.baseline import baseline_config, exported_metrics, snapshot
from tests.native_arc_benchmarks import path


class BaselineTests(unittest.TestCase):
    def test_nominal_length_changes_preferred_corner_radius(self):
        source = path([(150., 0.), (300., 0.), (300., 300.)])
        result = convert(source)
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertAlmostEqual(result.report['fillets'][0]['radius_mm'], 35.)

    def test_nozzle_dimensions_and_validation(self):
        self.assertEqual(Config().to_dict()['nozzle'], {'length_mm': 35., 'width_mm': 15.})
        self.assertEqual(Config.from_dict({'nozzle': {'length_mm': 30}}).nozzle.width_mm, 15.)
        for value in (0, -1, float('nan'), float('inf'), None, '15'):
            with self.subTest(value=value), self.assertRaises((ValueError, TypeError)):
                Config.from_dict({'nozzle': {'width_mm': value}})

    def test_modal_motion_comments_and_true_arc_length(self):
        source = ('G21 G90 G17\n(comment G1 X99)\nG0 X10 Y0 Z20\nM3\n'
                  'G1 X10.5 F2400\nX11\nG3 X9 Y0 I-1 J0\nG1 C10\nM5\n')
        metrics = exported_metrics(source, 2.)
        self.assertEqual(metrics['counts'], {'G0': 1, 'G1': 3, 'G2': 0, 'G3': 1})
        self.assertEqual(metrics['motion_blocks'], 5)
        self.assertEqual(metrics['linear_deposition_microblocks'], 2)
        # A semicircle of radius 1 has length pi, although its chord is only 2.
        self.assertEqual(exported_metrics(source, 2.5)['arc_deposition_microblocks'], 0)
        self.assertEqual(metrics['c_only_blocks'], 1)

    def test_reference_hashes_profile_and_display_independence(self):
        source = path([(float(i), 0.) for i in range(101)])
        config = baseline_config()
        result = convert(source, config)
        row = snapshot(result, source, config, .1)
        self.assertNotEqual(row['status'], 'FAIL', row['errors'])
        self.assertTrue(row['plan_export_count_match'])
        self.assertLess(row['export']['counts']['G1'], 100)
        self.assertIsNone(row['max_geometry_certificate_mm'])
        self.assertFalse(row['optimizer_run'])
        self.assertFalse(row['global_reference_extracted'])
        self.assertEqual(result.simulation['audit']['config']['nozzle'], {'length_mm': 35., 'width_mm': 15.})
        # Visualization samples must not change machine counts or the plan digest.
        result.samples.extend(result.samples[:3])
        other = snapshot(result, source, config, .2)
        self.assertEqual(row['export'], other['export'])
        self.assertEqual(row['baseline_plan_hash'], other['baseline_plan_hash'])
        wider = Config.from_dict(dict(config.to_dict(), nozzle={'length_mm': 35., 'width_mm': 16.}))
        self.assertNotEqual(snapshot(convert(source, wider), source, wider, .1)['config_hash'], row['config_hash'])


if __name__ == '__main__':
    unittest.main()
