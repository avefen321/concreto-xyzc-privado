import re
import unittest

from backend.core.config import Config
from backend.core.conversion import convert


TRIANGLE = '''G21 G90 G17
G0 Z70 ; MACHINE_SAFE_Z
G0 X20 Y20 Z70
G0 X170 Y20 Z70
G0 X170 Y20 Z0
G0 X245 Y149.904 Z0
G1 X200.311 Y227.308 Z0 F3000
G3 X139.689 Y227.308 Z0 R35
G1 X50.311 Y72.500 Z0
G3 X80.622 Y20 Z0 R35
G1 X259.378 Y20 Z0
G3 X289.689 Y72.500 Z0 R35
G1 X245 Y149.904 Z0
M30
'''


def configured(**sections):
    return Config.from_dict({
        'initial_xy_approach': {'enabled': True},
        'layer_transition': {'safe_seam': True, 'corner_guard_mm': 20,
                             'preferred_corner_guard_mm': 40},
        'orientation': {'mode': 'alignment'},
        **sections,
    })


class InitialXYApproachTests(unittest.TestCase):
    def test_triangle_approaches_near_origin_on_straight(self):
        result = convert(TRIANGLE, configured())
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        approach = result.report['initial_xy_approach']
        seam = result.report['safe_seams'][0]
        self.assertEqual(approach['removed_z_motion_count'], 2)
        self.assertAlmostEqual(approach['required_start_z_mm'], 20)
        self.assertAlmostEqual(approach['selected_start_xyz'][0], 170, places=3)
        self.assertAlmostEqual(approach['selected_start_xyz'][1], 20)
        self.assertEqual(seam['status'], 'PASS')
        self.assertGreaterEqual(seam['corner_distance_mm'], 40)
        self.assertTrue(seam['preferred_guard_met'])
        motions = [line for line in result.gcode.splitlines()
                   if re.match(r'^G[0123](?:\s|$)', line)]
        self.assertRegex(motions[0], r'^G0 X170\.000 Y20\.000 C0\.000$')
        self.assertNotIn(' Z', motions[0])
        self.assertFalse(any('Z70.000' in line for line in motions[:3]))
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')

    def test_cordon_still_joins_contour_after_startup_cleanup(self):
        source = '''G21 G90
G0 Z70 ; MACHINE_SAFE_Z
G0 X0 Y0 Z0
G1 X100 Y0 F600
G1 X100 Y100
G1 X0 Y100
G1 X0 Y0
'''
        result = convert(source, configured(cordon={'mode': 'add'},
                                            xy_boundary_offset={'enabled': True}))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertEqual(result.report['cordon']['validation']['status'], 'PASS')
        first_g0 = next(line for line in result.gcode.splitlines() if line.startswith('G0 '))
        self.assertNotIn(' Z', first_g0)
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')

    def test_repeated_layers_rise_at_the_new_seam(self):
        printable = [line for line in TRIANGLE.splitlines()
                     if line.startswith(('G1 X', 'G3 X'))]
        second = [line.replace('Z0', 'Z18') for line in printable]
        source = TRIANGLE.replace('M30\n', '') + (
            'G1 X245 Y149.904 Z18 F3000\n' + '\n'.join(second) + '\nM30\n')
        result = convert(source, configured())
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        seams = result.report['safe_seams']
        self.assertEqual([row['status'] for row in seams], ['PASS', 'PASS'])
        self.assertAlmostEqual(seams[0]['selected_xyz'][0], seams[1]['selected_xyz'][0])
        self.assertAlmostEqual(seams[0]['selected_xyz'][1], seams[1]['selected_xyz'][1])
        emitted = [sample for sample in result.samples if sample.emit]
        transitions = [(a, b) for a, b in zip(emitted, emitted[1:])
                       if b.generated_reason == 'LAYER_TRANSITION']
        self.assertTrue(transitions)
        for before, after in transitions:
            self.assertAlmostEqual(before.X, after.X)
            self.assertAlmostEqual(before.Y, after.Y)
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')

    def test_process_command_export_must_keep_source_approach(self):
        with self.assertRaisesRegex(ValueError, 'solo movimiento'):
            convert(TRIANGLE, configured(serialization={'motion_only': False}))

    def test_short_straight_uses_minimum_and_reports_preference_unmet(self):
        source = ('G21 G90\nG0 X0 Y0 Z0\nG1 X60 Y0 F600\n'
                  'G1 X60 Y60\nG1 X0 Y60\nG1 X0 Y0\n')
        result = convert(source, configured(corner_smoothing={
            'radius_mode': 'fixed', 'radius_mm': 10}))
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        seam = result.report['safe_seams'][0]
        self.assertEqual(seam['status'], 'WARNING')
        self.assertFalse(seam['preferred_guard_met'])
        self.assertGreaterEqual(seam['corner_distance_mm'], 20)
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')

    def test_closed_all_curve_path_starts_on_nearest_smooth_point(self):
        source = ('G21 G90 G17\nG0 X150 Y100 Z0\n'
                  'G3 X100 Y150 I-50 J0 F600\n'
                  'G3 X50 Y100 I0 J-50\n'
                  'G3 X100 Y50 I50 J0\n'
                  'G3 X150 Y100 I0 J50\n')
        result = convert(source, configured())
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        seam = result.report['safe_seams'][0]
        self.assertTrue(seam['curve_start'])
        self.assertTrue(seam['reindexed'])
        self.assertEqual(seam['selected_xyz'][:2], [50, 100])
        self.assertAlmostEqual(seam['curve_tangent_jump_deg'], 0)
        self.assertEqual(result.report['validators']['Serialization']['status'], 'PASS')


if __name__ == '__main__':
    unittest.main()
