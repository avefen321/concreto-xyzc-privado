import math
import unittest

from backend.core.config import Config
from backend.core.conversion import convert


OBLIQUE = (
    'G21 G90\n'
    'G0 X0 Y0 Z0\n'
    'M3\n'
    'G1 X50 Y86.602540378 F600\n'
    'M5\n'
)


def startup(axis=0, symmetry=180, motion_only=True, speed=10):
    return Config.from_dict({
        'orientation': {
            'mode': 'alignment',
            'alignment_offset_deg': 90,
            'symmetry_period_deg': symmetry,
            'initial_orientation_mode': 'ROTATE_AT_START',
            'home_axis_deg': axis,
            'initial_rotation_speed_deg_s': speed,
        },
        'serialization': {'motion_only': motion_only},
    })


class InitialOrientationTests(unittest.TestCase):
    def test_oblique_start_uses_selected_physical_home_axis(self):
        x = convert(OBLIQUE, startup(0))
        y = convert(OBLIQUE, startup(90))
        self.assertAlmostEqual(x.report['initial_c_reference']['target_c_deg'], -30, places=6)
        self.assertAlmostEqual(y.report['initial_c_reference']['target_c_deg'], 60, places=6)
        self.assertEqual(x.report['initial_c_reference']['home_axis_label'], '+X')
        self.assertEqual(y.report['initial_c_reference']['home_axis_label'], '+Y')
        # Both machine references produce the same physical nozzle line modulo 180°.
        px=x.report['initial_c_reference']['world_orientation_deg']
        py=y.report['initial_c_reference']['world_orientation_deg']
        self.assertAlmostEqual((px-py+90) % 180 - 90, 0, places=6)

    def test_arrive_rotate_restore_activate_then_print(self):
        result = convert(OBLIQUE, startup(0, motion_only=False))
        lines=result.gcode.splitlines()
        approach=next(i for i,line in enumerate(lines) if line.startswith('G0 X0.000 Y0.000'))
        exact=lines.index('G61.1')
        rotation=next(i for i,line in enumerate(lines) if line.startswith('G1 C-30.000'))
        restore=next(i for i,line in enumerate(lines) if i>rotation and line.startswith('G64'))
        activation=lines.index('M3')
        printing=next(i for i,line in enumerate(lines) if line.startswith('G1 X50.000'))
        self.assertLess(approach,exact)
        self.assertEqual([exact+1,rotation+1,restore+1,activation+1],
                         [rotation,restore,activation,printing])
        primitive=next(p for p in result.final_motion_plan.primitives
                       if p.generated_reason=='INITIAL_C_ALIGNMENT')
        self.assertEqual(primitive.type,'C_ONLY')
        self.assertEqual(primitive.process_state,'TRAVEL')
        self.assertEqual(primitive.start_xyz,primitive.end_xyz)
        self.assertIsNone(primitive.E)
        self.assertEqual(result.report['validators']['InitialAlignment']['status'],'PASS')

    def test_g94_rotary_feed_and_velocity_cap(self):
        result=convert(OBLIQUE,Config.from_dict({
            'orientation':{'mode':'alignment','alignment_offset_deg':90,
                           'symmetry_period_deg':180,'home_axis_deg':0,
                           'initial_orientation_mode':'ROTATE_AT_START',
                           'initial_rotation_speed_deg_s':12},
            'kinematics':{'max_c_velocity_deg_s':4}}))
        line=next(line for line in result.gcode.splitlines() if line.startswith('G1 C'))
        self.assertIn('F240.000',line)
        ref=result.report['initial_c_reference']
        self.assertEqual(ref['rotation_feed_deg_min'],240)
        sample=next(s for s in result.samples if s.generated_reason=='INITIAL_C_ALIGNMENT')
        self.assertAlmostEqual(abs(sample.omega_C_deg_s),4)

    def test_no_redundant_block_when_home_already_matches(self):
        # +Y travel with a perpendicular nozzle requires 180°, equivalent to C0
        # for a bidirectional nozzle home on +X.
        source='G21 G90\nG0 X0 Y0\nM3\nG1 X0 Y100 F600\nM5\n'
        result=convert(source,startup(0,180))
        self.assertFalse(any(s.generated_reason=='INITIAL_C_ALIGNMENT' for s in result.samples))
        self.assertEqual(result.report['initial_c_reference']['status'],'ALREADY_ALIGNED')
        self.assertNotIn('\nG61.1\n',result.gcode)

    def test_alignment_occurs_only_once_across_three_layers(self):
        rows=['G21 G90','G0 X0 Y0 Z0']
        for z in (0,10,20):
            rows += ['M3',f'G1 X100 Y0 Z{z} F600','G1 X100 Y100',
                     'G1 X0 Y100','G1 X0 Y0','M5']
            if z != 20: rows += [f'G1 Z{z+10}','G0 X0 Y0']
        result=convert('\n'.join(rows)+'\n',startup(0,180))
        alignments=[p for p in result.final_motion_plan.primitives
                    if p.generated_reason=='INITIAL_C_ALIGNMENT']
        self.assertEqual(len(alignments),1)
        for primitive in result.final_motion_plan.primitives:
            if primitive.type=='Z_ONLY':
                self.assertEqual(primitive.start_xyz[:2],primitive.end_xyz[:2])
                self.assertEqual(primitive.start_c,primitive.end_c)

    def test_old_boolean_profiles_keep_their_meaning(self):
        tangent=Config.from_dict({'orientation':{'home_aligned_start':True}})
        absolute=Config.from_dict({'orientation':{'home_aligned_start':False}})
        self.assertEqual(tangent.orientation.initial_orientation_mode,'HOME_ALIGNED_TANGENT')
        self.assertEqual(absolute.orientation.initial_orientation_mode,'ABSOLUTE_MODEL')
        self.assertFalse(any(s.generated_reason=='INITIAL_C_ALIGNMENT'
                             for s in convert(OBLIQUE,tangent).samples))

    def test_process_active_during_approach_is_rejected_when_preserved(self):
        source='G21 G90\nM3\nG0 X50 Y0\nG1 X100 Y0 F600\nM5\n'
        with self.assertRaisesRegex(ValueError,'antes del recorrido de aproximación'):
            convert(source,startup(0,180,motion_only=False))
        # In the normal geometry-only export material is external/manual, so
        # the generated motion remains valid and reports the source condition.
        result=convert(source,startup(0,180,motion_only=True))
        self.assertTrue(result.report['initial_c_reference']['source_activation_before_approach'])

    def test_g90_is_retained_as_absolute_coordinate_mode(self):
        result=convert(OBLIQUE,startup(90))
        self.assertIn('\nG90\n',result.gcode)
        self.assertIn('\nG1 C60.000 ',result.gcode)


if __name__ == '__main__':
    unittest.main()
