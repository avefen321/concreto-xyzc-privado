import math
import unittest
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import interpret, parse


def path(points, z=0):
    return '\n'.join(f'G1 X{x:.7f} Y{y:.7f} Z{z:.7f} F600' for x, y in points) + '\n'


class AuditFixTests(unittest.TestCase):
    def test_perpendicular_x_y_and_diagonal(self):
        for endpoint, expected in [((100, 0), 90), ((0, 100), 180), ((100, 100), 135)]:
            with self.subTest(endpoint=endpoint):
                result = convert('G21 G90\nM3\n' + path([endpoint]))
                deposition = [s for s in result.samples if s.deposition_active and s.emit]
                self.assertTrue(deposition)
                physical_c=deposition[-1].C_deg-deposition[-1].c_reference_shift_deg
                self.assertAlmostEqual(physical_c, expected, places=6)
                self.assertAlmostEqual((physical_c - deposition[-1].heading_unwrapped_deg) % 360,
                                       90, places=6)

    def test_square_four_circular_fillets(self):
        source = 'G21 G90\nG0 X0 Y0\nM3\n' + path([(100, 0), (100, 100), (0, 100), (0, 0)])
        result = convert(source, Config.legacy())
        self.assertEqual(result.report['status'], 'PASS')
        self.assertEqual(result.report['counts']['fillets'], 4)
        first = result.report['fillets'][0]
        self.assertEqual(first['status'], 'PASS')
        for actual, expected in zip(first['t1'], (90, 0, 0)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(first['t2'], (100, 10, 0)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(first['center'][0], 90, places=6)
        self.assertAlmostEqual(first['center'][1], 10, places=6)
        self.assertEqual(first['sweep_deg'], 90)
        self.assertAlmostEqual(first['c_start_deg'], 0, places=6)
        self.assertAlmostEqual(first['c_end_deg'], 90, places=6)
        self.assertLessEqual(first['corner_orientation_error_at_arc_exit_deg'], 0.001)
        self.assertEqual(first['corner_settling_distance_after_arc_mm'], 0)
        arc_c = [s.C_deg for s in result.samples if s.fillet_id == first['id'] and
                 s.stroke_id == first['stroke_id'] and s.generated_by_fillet]
        self.assertGreater(len(arc_c), 10)
        self.assertTrue(all(a <= b + 1e-9 for a, b in zip(arc_c, arc_c[1:])))
        self.assertTrue(any('G0 X10.000 Y0.000 Z0.000 C0.000' in line
                            for line in result.gcode.splitlines()))
        self.assertNotIn('M3', result.gcode)

    def test_preserve_path_remains_exact(self):
        cfg = Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'}})
        source = 'G21 G90\nM3\n' + path([(100, 0), (100, 100)])
        result = convert(source, cfg)
        self.assertFalse(result.report['fillets'])
        self.assertFalse(result.report['geometry']['modified_locally'])
        self.assertLess(result.report['geometry']['max_xy_deviation_mm'], 1e-6)
        self.assertTrue(any(s.X == 100 and s.Y == 0 for s in result.samples))

    def test_clockwise_fillet_has_negative_sweep_and_tangent_endpoints(self):
        source = 'G21 G90\nG0 X0 Y0\nM3\n' + path([(100, 0), (100, -100)])
        result = convert(source, Config.legacy())
        fillet = result.report['fillets'][0]
        self.assertEqual(fillet['status'], 'PASS')
        self.assertAlmostEqual(fillet['sweep_deg'], -90)
        self.assertAlmostEqual(fillet['t1'][0], 90)
        self.assertAlmostEqual(fillet['t1'][1], 0)
        self.assertAlmostEqual(fillet['t2'][0], 100)
        self.assertAlmostEqual(fillet['t2'][1], -10)
        self.assertAlmostEqual(fillet['c_start_deg'], 0)
        self.assertAlmostEqual(fillet['c_end_deg'], -90)
        self.assertLessEqual(fillet['corner_orientation_error_at_arc_exit_deg'], .001)

    def test_nearby_corners_do_not_overlap(self):
        source = 'G21 G90\nM3\n' + path([(20, 0), (20, 15), (40, 15)])
        result = convert(source, Config.legacy())
        statuses = [f['status'] for f in result.report['fillets']]
        self.assertIn('PASS', statuses)
        self.assertIn('FILLET_NOT_FEASIBLE', statuses)
        self.assertIn('FILLET_NOT_FEASIBLE', ' '.join(result.report['warnings']))

    def test_high_initial_travel_is_real_and_ui_can_start_at_deposition(self):
        source = ('G21\nG90\nG92 X0 Y0 Z0\nG0 Z30\nG0 X70 Y50\n'
                  'G0 Z10\nM3\nG1 X80 Y50 F600\n')
        result = convert(source, Config.legacy())
        self.assertEqual((result.samples[0].X, result.samples[0].Y, result.samples[0].Z), (0, 0, 30))
        start = next(s for s in result.samples if s.deposition_active)
        self.assertEqual((start.X, start.Y, start.Z), (70, 50, 10))
        self.assertEqual(result.simulation['samples'][result.simulation['deposition_start_index']]['Z'], 10)
        self.assertEqual(result.report['initial_trace'][0]['motion_type'], 'Z_ONLY')
        self.assertEqual(result.report['initial_sample_trace'][0]['Z'], 30)
        self.assertTrue(result.report['initial_sample_trace'][0]['is_z_only'])

    def test_three_layers_identical_with_and_without_smoothing(self):
        square = [(40, 0), (40, 40), (0, 40), (0, 0)]
        source = 'G21 G90\nG0 X0 Y0 Z10\n'
        for z in (10, 20, 30):
            if z != 10: source += f'G0 Z{z}\n'
            source += 'M3\n' + path(square, z) + 'M5\n'
        for mode in ('PRESERVE_PATH', 'SMOOTH_CORNERS'):
            with self.subTest(mode=mode):
                result = convert(source, Config.from_dict({'geometry': {'mode': mode}}))
                self.assertEqual(result.report['counts']['layers'], 3)
                self.assertLessEqual(result.report['geometry']['max_interlayer_xy_drift_mm'], 1e-8)
                self.assertEqual(len(result.report['geometry']['interlayer_comparisons']), 2)
                if mode == 'SMOOTH_CORNERS':
                    self.assertEqual(result.report['counts']['fillets'], 12)

    def test_g28_is_blocked_instead_of_misinterpreted(self):
        with self.assertRaisesRegex(ValueError, 'G28/G30'):
            interpret(parse('G0 X10\nG28 X0\n'))

    def test_smooth_curve_has_no_fillets(self):
        quarter = [(50 * math.cos(i * math.pi / 64), 50 * math.sin(i * math.pi / 64))
                   for i in range(1, 33)]
        source = 'G21 G90\nG0 X50 Y0\nM3\n' + path(quarter)
        result = convert(source)
        self.assertEqual(result.report['counts']['fillets'], 0)
        self.assertEqual(result.report['status'], 'PASS')

    def test_sine_curve_has_no_fillets(self):
        sine = [(float(x), 10 * math.sin(x / 20)) for x in range(2, 102, 2)]
        result = convert('G21 G90\nM3\n' + path(sine))
        self.assertEqual(result.report['counts']['fillets'], 0)
        self.assertEqual(result.report['status'], 'PASS')


if __name__ == '__main__':
    unittest.main()
