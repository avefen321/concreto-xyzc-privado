"""Automatic 20 mm XY clearance for the local conversion UI."""

from pathlib import Path
import re
import unittest

from backend.core.config import Config
from backend.core.conversion import convert
from tests.test_layer_completion import repeated_contour


ROOT = Path(__file__).resolve().parent.parent


def configured(**sections):
    return Config.from_dict({'xy_boundary_offset': {'enabled': True},
                             'geometry': {'mode': 'PRESERVE_PATH'},
                             'cordon': {'mode': 'legacy'}, **sections})


class XYBoundaryOffsetTests(unittest.TestCase):
    def assert_nonnegative_export(self, result):
        for line in result.gcode.splitlines():
            if not line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')):
                continue
            for value in re.findall(r'\b[XY](-?\d+(?:\.\d+)?)', line):
                self.assertGreaterEqual(float(value), 0, line)

    def test_existing_square_and_new_cordon_move_together(self):
        source = (ROOT / 'examples' / 'cuadrado.ngc').read_text()
        result = convert(source, configured(cordon={'mode': 'add'}))
        offset = result.report['xy_boundary_offset']
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertTrue(offset['applied'])
        self.assertAlmostEqual(offset['output_bounds_xy_mm']['min_x'], 20)
        self.assertAlmostEqual(offset['output_bounds_xy_mm']['min_y'], 20)
        self.assertGreater(offset['shift_x_mm'], 20)
        self.assertGreater(offset['shift_y_mm'], 20)
        self.assertEqual(result.report['cordon']['validation']['status'], 'PASS')
        self.assertAlmostEqual(result.report['cordon']['actual_start_xyz'][0], 20)
        self.assertAlmostEqual(result.report['cordon']['actual_start_xyz'][1], 20)
        self.assert_nonnegative_export(result)
        self.assertNotIn('M3', result.gcode)

    def test_axes_are_independent_and_positive_only_path_is_unchanged(self):
        source = 'G21 G90\nG0 X100 Y0 Z0\nG1 X200 Y0 F600\n'
        result = convert(source, configured())
        offset = result.report['xy_boundary_offset']
        self.assertEqual(offset['shift_x_mm'], 0)
        self.assertEqual(offset['shift_y_mm'], 20)
        self.assertAlmostEqual(offset['output_bounds_xy_mm']['min_y'], 20)
        self.assert_nonnegative_export(result)
        positive = convert('G21 G90\nG0 X100 Y100 Z0\nG1 X200 Y100 F600\n', configured())
        self.assertFalse(positive.report['xy_boundary_offset']['applied'])
        self.assertEqual(positive.report['xy_boundary_offset']['shift_x_mm'], 0)
        self.assertEqual(positive.report['xy_boundary_offset']['shift_y_mm'], 0)

    def test_native_arc_extremum_beyond_endpoint_triggers_offset(self):
        source = 'G21 G90 G17\nG0 X5 Y10 Z0\nG2 X5 Y30 I0 J10 F600\n'
        result = convert(source, configured())
        offset = result.report['xy_boundary_offset']
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(offset['shift_x_mm'], 25)
        self.assertEqual(offset['shift_y_mm'], 0)
        self.assertAlmostEqual(offset['output_bounds_xy_mm']['min_x'], 20)
        self.assertIn('G2 ', result.gcode)
        self.assert_nonnegative_export(result)

    def test_first_g1_has_rapid_to_shifted_start(self):
        source = 'G21 G90\nG1 X100 Y0 Z0 F600\nG1 X100 Y100\n'
        result = convert(source, configured(
            z_conversion={'enabled': False},
            orientation={'initial_orientation_mode': 'HOME_ALIGNED_TANGENT'},
            native_arcs={'enabled': False}, arc_fit={'enabled': False}))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        moves = [line for line in result.gcode.splitlines()
                 if line.startswith(('G0 ', 'G1 '))]
        self.assertTrue(moves[0].startswith('G0 X20.000 Y20.000'), moves[:3])
        self.assertTrue(any(line.startswith('G1 X120.000 Y20.000') for line in moves))
        self.assert_nonnegative_export(result)

    def test_completed_layers_and_cordon_use_same_translation(self):
        result = convert(repeated_contour(), configured(
            cordon={'mode': 'add'}, layer_completion={'decision': 'complete'}))
        offset = result.report['xy_boundary_offset']
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertTrue(offset['applied'])
        self.assertEqual(result.report['cordon']['validation']['status'], 'PASS')
        self.assertEqual(result.report['layer_completion']['validation']['status'], 'PASS')
        self.assert_nonnegative_export(result)


if __name__ == '__main__':
    unittest.main()
