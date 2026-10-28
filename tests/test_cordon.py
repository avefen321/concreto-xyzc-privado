import math
from pathlib import Path
import unittest

from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.layer_completion import review_source


ROOT = Path(__file__).resolve().parent.parent
EXISTING = '\n'.join((
    'G21', 'G90', 'G0 X-20 Y-20 Z0', 'M3', 'G1 X0 Y0 F600',
    'G1 X100 Y0', 'G1 X100 Y100', 'G1 X0 Y100', 'G1 X0 Y0', 'M5', 'M2',
)) + '\n'


def selected(mode, closure='preserve'):
    return Config.from_dict({'cordon': {'mode': mode},
                             'layer_completion': {'decision': closure}})


class CordonTests(unittest.TestCase):
    def test_zero_length_first_g1_does_not_block_no_cordon(self):
        source = '\n'.join((
            'G21', 'G90', 'G17',
            'G0 X0 Y0 Z10 F2000',
            'G1 X0 Y0 Z10 F1200',
            'G1 X10 Y15.451 Z10 F1200',
            'G1 X20 Y29.389 Z10 F1200',
        )) + '\n'
        review = review_source(source, Config())['cordon']
        self.assertTrue(review['can_omit'])
        self.assertTrue(review['can_add'])
        self.assertEqual(review['contour_start_xyz'], [0.0, 0.0, 10.0])
        result = convert(source, selected('omit'))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(result.report['cordon']['status'], 'ABSENT')
        self.assertFalse(any(s.cordon_role == 'CORDON_PRIME' for s in result.samples))

    def test_sine_wall_with_alternating_layers_converts_without_cordon(self):
        source = (ROOT / 'pared_senoidal_concreto_3d.gcode').read_text(encoding='utf-8')
        review = review_source(source, Config())['cordon']
        self.assertTrue(review['can_omit'])
        result = convert(source, selected('omit'))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(result.report['cordon']['status'], 'ABSENT')
        self.assertEqual(result.report['counts']['layers'], 10)

    def test_add_45_degree_cordon_to_square_without_existing_entry(self):
        source = (ROOT / 'examples' / 'cuadrado.ngc').read_text()
        review = review_source(source, Config())
        self.assertFalse(review['review_required'])
        self.assertTrue(review['cordon']['can_add'])
        self.assertFalse(review['cordon']['can_keep'])
        self.assertTrue(review['cordon']['negative_xy'])
        result = convert(source, selected('add'))
        cordon = result.report['cordon']
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(cordon['status'], 'ADDED')
        self.assertEqual(cordon['validation']['status'], 'PASS')
        self.assertAlmostEqual(cordon['validation']['actual_length_mm'], 65)
        start, end = cordon['actual_start_xyz'], cordon['actual_end_xyz']
        self.assertAlmostEqual(abs(start[0] - end[0]), 65 / math.sqrt(2))
        self.assertAlmostEqual(abs(start[1] - end[1]), 65 / math.sqrt(2))
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')
        lines = result.gcode.splitlines()
        primer = next(i for i, line in enumerate(lines) if line.startswith('G1 X0.000 Y0.000 Z20.000'))
        self.assertNotIn('M3', result.gcode)
        self.assertNotIn('M5', result.gcode)
        self.assertTrue(lines[primer + 1].startswith('G0 X0.000 Y0.000 Z20.000 C90.000'))

    def test_existing_priming_entry_can_be_adjusted_kept_or_removed(self):
        review = review_source(EXISTING, Config())['cordon']
        self.assertTrue(all(review[key] for key in ('existing', 'can_add', 'can_keep', 'can_omit')))
        for mode, expected in (('add', 'ADJUSTED'), ('keep', 'KEPT'), ('omit', 'OMITTED')):
            with self.subTest(mode=mode):
                result = convert(EXISTING, selected(mode))
                cordon = result.report['cordon']
                self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
                self.assertEqual(cordon['status'], expected)
                self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')
                self.assertNotIn('M3', result.gcode)
                self.assertNotIn('M5', result.gcode)
                if mode == 'omit':
                    self.assertFalse(any(s.cordon_role == 'CORDON_PRIME' for s in result.samples))
                else:
                    self.assertEqual(cordon['validation']['status'], 'PASS')
                    target = 65 if mode == 'add' else math.hypot(20, 20)
                    self.assertAlmostEqual(cordon['validation']['actual_length_mm'], target)

    def test_cordon_and_first_layer_completion_are_independent(self):
        source = (ROOT / 'cruz_CA.gcode').read_text()
        review = review_source(source, Config())
        self.assertTrue(review['review_required'])
        self.assertTrue(review['cordon']['can_add'])
        result = convert(source, selected('add', 'complete'))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(result.report['cordon']['validation']['status'], 'PASS')
        self.assertEqual(result.report['layer_completion']['validation']['status'], 'PASS')
        self.assertEqual(result.report['layer_completion']['added_closure_count'], 1)

    def test_selected_mode_exports_geometry_without_extrusion_commands(self):
        source = 'G21 G90\nG0 X0 Y0 Z0\nM3 S100\nG1 X100 Y0 E5 F600\nM5\nM7 P2\nM2\n'
        config = Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'},
                                   'cordon': {'mode': 'omit'}})
        result = convert(source, config)
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        lines = result.gcode.splitlines()
        self.assertTrue(any(line.startswith('G1 X100.000 Y0.000') for line in lines))
        self.assertFalse(any(line.startswith(('M3', 'M4', 'M5', 'M7', 'S', 'P', 'T')) or
                             ' E' in line for line in lines))
        self.assertEqual(lines[-1], 'M2')

    def test_motion_only_is_default_for_direct_conversion(self):
        source = 'G21 G90\nM3 S100\nG4 P1\nG1 X100 Y0 F600\nM5\n'
        result = convert(source)
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertFalse(any(line.startswith(('M3', 'M4', 'M5', 'G4', 'S', 'P'))
                             for line in result.gcode.splitlines()))


if __name__ == '__main__':
    unittest.main()
