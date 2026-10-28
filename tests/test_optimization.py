"""Regressions added before the optimization (red baseline recorded in audit)."""
import json
import math
import unittest
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse, interpret
from backend.core.path import build
from backend.core.orientation import Field, nearest_equivalent_angle
from backend.core.fillet import optimize_stroke


def path(points, feed=600, close=False):
    points = list(points)
    if close:
        points.append(points[0])
    first = points[0]
    return ('G21 G90\nG0 X%.9f Y%.9f Z0\nM3\n' % first +
            '\n'.join('G1 X%.9f Y%.9f F%g' % (*p, feed) for p in points[1:]) + '\nM5\n')


SQUARE = path([(0, 0), (100, 0), (100, 100), (0, 100)], close=True)
RIGHT = path([(0, 0), (100, 0), (100, 100)])


class OptimizationTests(unittest.TestCase):
    def test_analytic_arc_orientation_and_curvature(self):
        cfg = Config.from_dict({'sampling': {'max_delta_c_deg': 0.3}, 'corner_smoothing':{'radius_mode':'fixed','radius_mm':10}})
        r = convert(RIGHT, cfg)
        f = r.report['fillets'][0]
        arc = [s for s in r.samples if s.generated_by_fillet and s.emit]
        angles = []
        for s in arc:
            angle = math.degrees(math.atan2(s.Y-f['center'][1], s.X-f['center'][0]))
            physical_c=s.C_deg-s.c_reference_shift_deg
            expected = nearest_equivalent_angle(angle+180, physical_c, 360)
            self.assertAlmostEqual(physical_c, expected, places=7)
            self.assertAlmostEqual(s.curvature_1_mm, 0.1, places=7)
            self.assertAlmostEqual(math.hypot(s.X-f['center'][0], s.Y-f['center'][1]), 10, places=7)
            angles.append(angle)
        self.assertTrue(all(a <= b for a,b in zip(angles, angles[1:])))

    def test_fillet_metrics_and_boundaries(self):
        r = convert(RIGHT, Config.from_dict({"corner_smoothing":{"radius_mode":"fixed","radius_mm":10}}))
        f = r.report['fillets'][0]
        self.assertAlmostEqual(f['max_polyline_deviation_mm'], 10*(1-math.sqrt(.5)))
        self.assertAlmostEqual(f['vertex_setback_mm'], 10*(math.sqrt(2)-1))
        self.assertEqual(f['corner_settling_distance_after_T2_mm'], 0)
        after = [s for s in r.samples if s.deposition_active and s.Y > 10]
        self.assertTrue(all(abs((s.C_deg-s.c_reference_shift_deg)-180)<1e-8 for s in after))

    def test_nonuniform_circle_is_smooth(self):
        angles=[0,.03,.08,.14,.22,.31,.43,.58,.77,1.,1.3,1.57]
        r=convert(path([(50*math.cos(a),50*math.sin(a)) for a in angles]))
        self.assertFalse(r.report['corners'])
        self.assertFalse(r.report['fillets'])
        for s in r.samples:
            if s.deposition_active and 2 < s.s_stroke < 70:
                target=math.degrees(math.atan2(s.Y,s.X))+180
                physical_c=s.C_deg-s.c_reference_shift_deg
                self.assertLess(abs(nearest_equivalent_angle(target,physical_c,360)-physical_c),1.0)

    def test_simplification_both_tolerances(self):
        pts=[(i*.1,5*math.sin(i*.1/20)) for i in range(1001)]
        cfg=Config.from_dict({'simplification': {'enabled':True,'geometry_tolerance_mm':.03,'orientation_tolerance_deg':.08}})
        r=convert(path(pts),cfg)
        m=r.report['simplification']
        self.assertLess(m['output_points'], m['input_points']/2)
        self.assertLessEqual(m['max_geometry_error_mm'],.03+1e-8)
        self.assertLessEqual(m['max_orientation_error_deg'],.08+1e-8)
        self.assertEqual(r.report['validators']['Sampling']['status'],'PASS')

    def test_safe_seam_and_z_only(self):
        source=SQUARE+'G0 Z10\nM3\nG1 X100 Y0 F600\nG1 X100 Y100\nG1 X0 Y100\nG1 X0 Y0\nM5\n'
        # A 100 mm square with R30 leaves 40 mm for the two 20 mm guards.
        r=convert(source,Config.from_dict({'nozzle':{'length_mm':30},'layer_transition':{'safe_seam':True,'corner_guard_mm':20}}))
        self.assertTrue(all(s['status']=='PASS' for s in r.report['safe_seams']))
        self.assertTrue(all(s['corner_distance_mm']>=20 for s in r.report['safe_seams']))
        emitted=[s for s in r.samples if s.emit]
        z=[(a,b) for a,b in zip(emitted,emitted[1:]) if b.generated_reason=='LAYER_TRANSITION']
        self.assertTrue(z)
        for a,b in z:
            self.assertEqual((a.X,a.Y),(b.X,b.Y))
        self.assertLessEqual(r.report['geometry']['max_interlayer_xy_drift_mm'],1e-8)

    def test_nonmotion_commands_preserved_once(self):
        r=convert('G21 G90\nS120\nM7 P2\nG1 X20 F600\nM5\nM2\n',
                  Config.from_dict({'serialization': {'motion_only': False}}))
        self.assertEqual(r.gcode.count('M7'),1)
        self.assertIn('S120',r.gcode)
        self.assertIn('M7 P2',r.gcode)

    def test_normalized_feed_and_relative_extrusion(self):
        r=convert('G20 G90 M83\nF10\nG1 X1 E1\nG1 X2 E1\n',Config.from_dict({'geometry':{'mode':'PRESERVE_PATH'},
                                                                                   'serialization':{'motion_only':False}}))
        self.assertNotIn('\nM83\n',r.gcode)
        self.assertIn('\nF254.000\n',r.gcode)
        motions,_=interpret(parse(r.gcode))
        self.assertAlmostEqual(motions[-1].e_end,50.8)

    def test_process_boundary_without_travel(self):
        r=convert('G21 G90\nM3\nG1 X20 F600\nM5\nM3\nG1 Y20\n')
        self.assertEqual(len([s for s in r.report['strokes'] if s['deposition']]),2)
        self.assertFalse(r.report['fillets'])

    def test_feed_policies_and_limits(self):
        cfg={'kinematics':{'max_c_velocity_deg_s':10,'max_c_acceleration_deg_s2':20},
             'corner_smoothing':{'max_radius_mm':30}}
        preserve=convert(RIGHT,Config.from_dict(cfg))
        self.assertTrue(all(s.feed_mm_min==600 for s in preserve.samples if s.deposition_active))
        self.assertEqual(preserve.report['feed_plan']['policy'],'PRESERVE_SOURCE')
        cfg['process']={'feed_policy':'ADAPTIVE_EXPERIMENTAL'}
        r=convert(RIGHT,Config.from_dict(cfg))
        self.assertLessEqual(r.report['kinematics']['max_omega_c_deg_s'],10+1e-6)
        self.assertLessEqual(r.report['kinematics']['max_alpha_c_deg_s2'],20+1e-6)
        self.assertTrue(any(s.feed_mm_min<600 for s in r.samples if s.deposition_active))

    def test_unknown_feed_not_invented(self):
        r=convert('G21 G90\nM3\nG1 X100\nG1 Y100\n',Config.from_dict(
            {'orientation':{'initial_orientation_mode':'HOME_ALIGNED_TANGENT'}}))
        self.assertTrue(all(s.feed_mm_min is None for s in r.samples))
        self.assertEqual(r.report['feed_plan']['unknown_feed_segments']>0,True)
        self.assertIsNone(r.report['kinematics']['max_omega_c_deg_s'])

    def test_single_final_dataset_and_hashes(self):
        a,b=convert(SQUARE),convert(SQUARE)
        self.assertEqual(a.gcode,b.gcode)
        self.assertEqual(a.simulation,b.simulation)
        self.assertEqual(a.report['conversion_id'],a.simulation['conversion_id'])
        self.assertEqual([s.to_dict() for s in a.samples],a.simulation['samples'])
        self.assertEqual([s.sample_id for s in a.samples if s.emit],
                         [x['sample_id'] for x in a.report['serialization_map'] if x['sample_id'] is not None])
        self.assertEqual([s['id'] for s in a.report['stages']],[f'S{i}' for i in range(9)])
        self.assertEqual(len(a.report['input_hash']),64)
        self.assertTrue('orientation_plan' in a.simulation)
        self.assertEqual(len(a.simulation['canonical_motion_program']),a.report['counts']['motions'])
        json.dumps(a.simulation,allow_nan=False)

    def test_vector_orientation(self):
        r=convert(RIGHT)
        for s in r.samples:
            if s.tangent_xy is not None:
                t,n=s.tangent_xy,s.nozzle_vector_xy
                self.assertAlmostEqual(math.hypot(*t),1)
                self.assertAlmostEqual(math.hypot(*n),1)
                self.assertAlmostEqual(t[0]*n[0]+t[1]*n[1],0,places=7)

    def test_manual_home_removes_initial_c90_but_keeps_g90(self):
        source='G21 G90\nG0 X0 Y0 Z0\nM3\nG1 X100 Y0 F600\nM5\n'
        result=convert(source,Config.from_dict(
            {'orientation':{'initial_orientation_mode':'HOME_ALIGNED_TANGENT'}}))
        self.assertIn('\nG90\n',result.gcode)
        self.assertNotIn('C90.000',result.gcode)
        first=next(s for s in result.samples if s.deposition_active)
        self.assertAlmostEqual(first.C_deg,0.)
        self.assertAlmostEqual(first.c_reference_shift_deg,-90.)

    def test_absolute_model_remains_available_explicitly(self):
        source='G21 G90\nG0 X0 Y0 Z0\nM3\nG1 X100 Y0 F600\nM5\n'
        result=convert(source,Config.from_dict(
            {'orientation':{'initial_orientation_mode':'ABSOLUTE_MODEL'}}))
        self.assertIn('C90.000',result.gcode)
        self.assertEqual(result.report['initial_c_reference']['mode'],'ABSOLUTE_MODEL')

    def test_dense_straight_collapses_without_flattening_a_circle(self):
        straight=convert(path([(float(i),0.) for i in range(101)]))
        printed=[p for p in straight.final_motion_plan.primitives if p.process_state=='DEPOSIT']
        self.assertEqual(len(printed),1)
        self.assertEqual(printed[0].type,'LINE')
        circle=convert(path([(50*math.cos(2*math.pi*i/80),50*math.sin(2*math.pi*i/80))
                             for i in range(80)],close=True))
        curved=[p for p in circle.final_motion_plan.primitives if p.process_state=='DEPOSIT']
        self.assertGreater(len(curved),1)
        self.assertTrue(any(p.type=='ARC' for p in curved))

    def test_open_layers_alternate_without_xy_or_c_travel(self):
        source='''G21 G90 G17 G91.1\nG0 X0 Y0 Z0\nM3\nG1 X50 Y0 F600\nG2 X100 Y50 I0 J50\nM5\nG1 Z10\nG0 X0 Y0\nM3\nG1 X50 Y0 F600\nG2 X100 Y50 I0 J50\nM5\nG1 Z20\nG0 X0 Y0\nM3\nG1 X50 Y0 F600\nG2 X100 Y50 I0 J50\nM5\n'''
        result=convert(source,Config.from_dict({'orientation':{'symmetry_period_deg':180},
                                                'layer_transition':{'alternate_open_paths':True}}))
        rows=result.report['open_path_transitions']
        self.assertEqual([r['reversed'] for r in rows],[False,True,False])
        self.assertFalse(any(p.type=='TRAVEL' and
                             (math.dist(p.start_xyz[:2],p.end_xyz[:2])>1e-8 or
                              abs(p.start_c-p.end_c)>1e-8)
                             for p in result.final_motion_plan.primitives))
        layers={layer:[] for layer in (1,2,3)}
        for p in result.final_motion_plan.primitives:
            if p.process_state=='DEPOSIT': layers[p.layer_id].append(p)
        self.assertEqual(layers[1][-1].end_xyz[:2],layers[2][0].start_xyz[:2])
        self.assertEqual(layers[2][-1].end_xyz[:2],layers[3][0].start_xyz[:2])

    def test_large_payload_does_not_drop_samples(self):
        r=convert('G0 X100 Y0\nG3 X0 Y100 I-100 J0 F600\n',Config.from_dict({'sampling':{'max_linear_step_mm':.006},'simplification':{'enabled':False}}))
        self.assertGreater(len(r.samples),20000)
        self.assertEqual(len(r.samples),len(r.simulation['samples']))

    def test_nonfinite_config_rejected(self):
        for data in ({'orientation':{'calibration_offset_deg':float('nan')}},
                     {'geometry':{'max_xy_deviation_mm':0}}, {'unknown_section':{}}):
            with self.subTest(data=data),self.assertRaises(ValueError):
                Config.from_dict(data)

    def test_fillet_over_90_degrees_with_180_symmetry(self):
        source=path([(0,0),(100,0),(50,86.602540378)])
        cfg=Config.from_dict({'orientation':{'symmetry_period_deg':180},'corner_smoothing':{'radius_mm':4}})
        r=convert(source,cfg)
        self.assertNotEqual(r.report['status'],'FAIL',r.report['errors'])
        f=r.report['fillets'][0]
        self.assertAlmostEqual(f['c_end_deg']-f['c_start_deg'],120,places=5)

    def test_small_native_arc_is_a_curve(self):
        r=convert('G21 G90\nG0 X1 Y0\nM3\nG3 X0 Y1 I-1 J0 F60\nM5\n')
        self.assertFalse(r.report['corners'])
        self.assertFalse(r.report['fillets'])

    def test_cyclically_reindexed_layers_are_compared(self):
        source=SQUARE+'G0 Z10\nG0 X100 Y0\nM3\nG1 X100 Y100 F600\nG1 X0 Y100\nG1 X0 Y0\nG1 X100 Y0\nM5\n'
        r=convert(source,Config.from_dict({'layer_transition':{'safe_seam':True}}))
        self.assertEqual(len(r.report['geometry']['interlayer_comparisons']),1)
        self.assertLessEqual(r.report['geometry']['max_interlayer_xy_drift_mm'],1e-7)


if __name__=='__main__': unittest.main()
