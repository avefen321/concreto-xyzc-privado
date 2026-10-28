import math
import unittest
from backend.core.config import Config
from backend.core.gcode import parse, interpret
from backend.core.conversion import convert, _validate_sample_geometry
from backend.core.orientation import nearest_equivalent_angle
from backend.core.path import build


def program(points, header='G21\nG90\nM3\n', footer=''):
    return header + '\n'.join(f'G1 X{x:.7f} Y{y:.7f} Z{z:.7f} F600' for x, y, z in points) + '\n' + footer


class ParserTests(unittest.TestCase):
    def test_comments_and_modal_relative(self):
        source = 'G21 G90 (units)\nG1 X10 Y5 F600 ; print\nG91\nG1 X2 Y-3\n'
        blocks = parse(source)
        self.assertIn('; print', blocks[1].comments)
        motions, warnings = interpret(blocks)
        self.assertEqual(motions[-1].end, (12, 2, 0))
        self.assertEqual(warnings, [])

    def test_g92_is_not_a_motion(self):
        motions, _ = interpret(parse('G21 G90\nG1 X10 F500\nG92 X0\nG1 X5\n'))
        self.assertEqual(len(motions), 2)
        self.assertEqual(motions[-1].start[0], 10)
        self.assertEqual(motions[-1].end[0], 15)

    def test_bad_token_is_fatal(self):
        with self.assertRaises(ValueError): parse('G1 Xnan')

    def test_lowercase_scientific_not_mistaken_for_extrusion(self):
        blocks = parse('G1 X3.0e-15 Y0')
        self.assertAlmostEqual(blocks[0].value('X'), 3e-15)
        self.assertIsNone(blocks[0].value('E'))

    def test_z_only_has_no_stroke(self):
        motions, _ = interpret(parse('G1 X10 F600\nG1 Z5\n'))
        strokes, _, _ = build(motions, Config())
        self.assertEqual(motions[-1].kind, 'Z_ONLY')
        self.assertIsNone(motions[-1].stroke_id)
        self.assertEqual(len(strokes), 1)

    def test_inch_units_and_e_axis(self):
        motions, _ = interpret(parse('G20 G90 M82\nG1 X1 F10 E1\nG91\nG1 X1 E2\n'))
        self.assertEqual(motions[0].end[0], 25.4)
        self.assertEqual(motions[0].feed, 254)
        self.assertEqual(motions[-1].end[0], 50.8)
        self.assertEqual(motions[-1].e_end, 50.8)

    def test_arc_is_linearized(self):
        source='G21 G90\nG0 X10 Y0\nM3\nG3 X0 Y10 I-10 J0 F600\n'
        result=convert(source, Config.from_dict({'native_arcs': {'enabled': False}}))
        self.assertGreater(len(result.samples),2)
        self.assertLess(len(result.samples),20)
        self.assertIn('G1 X0.000 Y10.000',result.gcode)
        self.assertNotIn('\nG3 ',result.gcode)


class ConversionTests(unittest.TestCase):
    def test_line_constant_orientation(self):
        result = convert(program([(0, 0, 0), (100, 0, 0)]))
        self.assertEqual(result.report['geometry']['status'], 'PASS')
        self.assertFalse(result.report['corners'])
        cs = [s.C_deg for s in result.samples if s.deposition_active]
        self.assertTrue(all(abs(c-90) < 1e-8 for c in cs))
        self.assertEqual(result.report['initial_c_reference']['mode'],'ROTATE_AT_START')
        self.assertEqual(result.report['initial_c_reference']['target_c_deg'],90)

    def test_square_four_corners_settle_at_vertex(self):
        points = [(0,0,0),(100,0,0),(100,100,0),(0,100,0),(0,0,0)]
        result = convert(program(points), Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'}}))
        corners = result.report['corners']
        self.assertEqual(len(corners), 4)
        self.assertTrue(all(c['category'] == 'SHARP_CORNER' for c in corners))
        self.assertTrue(all(c['settling_distance_after_vertex_mm'] == 0 for c in corners))
        for c in corners:
            vertex = next(s for s in result.samples if s.stroke_id == c['stroke_id'] and
                          abs(s.s_stroke - c['s_vertex']) < 1e-8 and s.emit)
            physical_c=vertex.C_deg-vertex.c_reference_shift_deg
            self.assertLess(abs((physical_c - c['outgoing_heading_deg'] - 90 + 180) % 360 - 180), 1e-6)
        self.assertEqual(result.report['geometry']['max_layer_xy_drift_mm'], 0)

    def test_two_identical_layers_no_xy_offset(self):
        square = [(0,0),(40,0),(40,40),(0,40),(0,0)]
        source = 'G21 G90\nM3\n' + '\n'.join(f'G1 X{x} Y{y} Z0 F600' for x,y in square) + '\nM5\nG0 Z10\nM3\n' + '\n'.join(f'G1 X{x} Y{y} Z10 F600' for x,y in square) + '\n'
        result = convert(source)
        self.assertEqual(result.report['counts']['layers'], 2)
        self.assertLessEqual(result.report['geometry']['max_layer_xy_drift_mm'], 1e-8)
        self.assertEqual(result.report['layers'][0]['original_bbox_xy'], result.report['layers'][1]['original_bbox_xy'])
        self.assertEqual(result.report['layers'][0]['converted_bbox_xy'], result.report['layers'][1]['converted_bbox_xy'])

    def test_smooth_circle_has_no_sharp_corners(self):
        points = [(50*math.cos(i*math.pi/64),50*math.sin(i*math.pi/64),0) for i in range(33)]
        result = convert(program(points, header='G21\nG90\nG0 X50 Y0\nM3\n'))
        self.assertFalse(any(c['category']=='SHARP_CORNER' for c in result.report['corners']))
        cs = [s.C_deg for s in result.samples if s.deposition_active]
        self.assertGreater(max(cs)-min(cs), 80)

    def test_sine_inflection_remains_continuous(self):
        points = [(i, 10*math.sin(i/20),0) for i in range(0,101,2)]
        result = convert(program(points))
        self.assertFalse(any(c['category']=='SHARP_CORNER' for c in result.report['corners']))
        self.assertEqual(result.report['status'], 'PASS')

    def test_irregular_curve_heading(self):
        angles = [0,.03,.08,.14,.22,.31,.43,.58,.77,1.0,1.3,1.57]
        points = [(50*math.cos(a),50*math.sin(a),0) for a in angles]
        result = convert(program(points, header='G21\nG90\nG0 X50 Y0\nM3\n'))
        self.assertEqual(result.report['geometry']['status'], 'PASS')
        self.assertFalse(any(c['category']=='SHARP_CORNER' for c in result.report['corners']))

    def test_infeasible_corner_recommends_feed(self):
        cfg = Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'},
                                'kinematics':{'max_c_velocity_deg_s':15,'max_c_acceleration_deg_s2':60}})
        result = convert(program([(0,0,0),(3,0,0),(3,50,0)]), cfg)
        corner = result.report['corners'][0]
        self.assertFalse(corner['feasible'])
        self.assertLess(corner['recommended_feed_mm_min'],600)
        self.assertIn('CORNER_NOT_KINEMATICALLY_FEASIBLE', ' '.join(result.report['warnings']))
        self.assertEqual(corner['settling_distance_after_vertex_mm'],0)

    def test_angular_wrap_and_multiturn(self):
        values = [170,175,179,-179,-175]
        unwrapped=[values[0]]
        for value in values[1:]: unwrapped.append(nearest_equivalent_angle(value,unwrapped[-1],360))
        self.assertEqual(unwrapped,[170,175,179,181,185])
        self.assertEqual(nearest_equivalent_angle(20,740,360),740)

    def test_geometry_drift_is_blocking(self):
        result = convert(program([(0,0,0),(10,0,0)]))
        sample = next(s for s in result.samples if s.emit and s.deposition_active)
        sample.X += 1
        motions, _ = interpret(parse(program([(0,0,0),(10,0,0)])))
        strokes, _, _ = build(motions, Config())
        _, _, errors = _validate_sample_geometry(result.samples,strokes,Config())
        self.assertTrue(any('FAIL_GEOMETRY_DRIFT' in error for error in errors))

    def test_travel_does_not_join_print_strokes(self):
        source='G21 G90\nM3\nG1 X10 Y0 F600\nM5\nG0 X20\nM3\nG1 X20 Y10\n'
        result=convert(source)
        printed=[s for s in result.report['strokes'] if s['deposition']]
        self.assertEqual(len(printed),2)
        self.assertEqual(result.report['counts']['layers'],1)

    def test_exact_vertex_requires_opt_in(self):
        cfg=Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'},
                              'corner_planner':{'strategy':'exact_vertex'}})
        result=convert(program([(0,0,0),(20,0,0),(20,20,0)]),cfg)
        self.assertEqual(result.report['status'],'FAIL')
        self.assertEqual(result.gcode,'')
        cfg.corner_planner.allow_c_only_at_sharp_corner=True
        result=convert(program([(0,0,0),(20,0,0),(20,20,0)]),cfg)
        self.assertIn('G1 C',result.gcode)

    def test_e_is_interpolated_and_absolute(self):
        source='G21 G90 M82\nG1 X10 E10 F600\n'
        result=convert(source, Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'},'sampling':{'mode':'legacy','max_linear_step_mm':5},
                                                 'serialization':{'motion_only':False}}))
        self.assertIn('\nM82\n',result.gcode)
        e_values=[s.E for s in result.samples if s.emit and s.E is not None]
        self.assertGreater(len(e_values),1)
        self.assertEqual(e_values[-1],10)
        self.assertTrue(all(a<=b for a,b in zip(e_values,e_values[1:])))

    def test_unknown_command_keeps_parameters(self):
        result=convert('G21 G90\nG4 P1\nG1 X10 F600\n',
                       Config.from_dict({'serialization':{'motion_only':False}}))
        self.assertIn('\nG4 P1\n',result.gcode)
        self.assertTrue(any('G4' in warning for warning in result.report['warnings']))


if __name__ == '__main__': unittest.main()
