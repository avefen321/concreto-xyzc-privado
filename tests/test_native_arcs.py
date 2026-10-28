import json
import math
import unittest
from dataclasses import replace
from pathlib import Path
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse, interpret, arc_geometry, motion_extent_points
from backend.core.arc_fitting import biarc_hermite, circle_through, ArcFitter, merge_fitted_arcs
from backend.core.orientation import Field
from backend.core.path import Point, Stroke, build
from backend.core.arc_serialization import arc_words
from tests.native_arc_benchmarks import fixtures, path


class NativeArcTests(unittest.TestCase):
    def run_case(self,source,config=None):
        result=convert(source,config)
        self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
        self.assertEqual(result.report['validators']['NativeArcs']['status'],'PASS')
        return result

    def arcs(self,result): return [p for p in result.final_motion_plan.primitives if p.type=='ARC']

    def test_R30_and_IJ_equivalent_one_machine_arc(self):
        source=path([(150,0),(300,0),(300,300)])
        outputs=[]
        for format in ('AUTO','RADIUS_R','CENTER_IJK'):
            result=self.run_case(source,Config.from_dict({'nozzle':{'length_mm':30},'arc_output':{'format':format},'orientation':{'alignment_offset_deg':0}}))
            arc,=self.arcs(result)
            self.assertEqual(arc.start_xyz[:2],(270.,0.))
            self.assertAlmostEqual(arc.end_xyz[0],300.); self.assertAlmostEqual(arc.end_xyz[1],30.)
            self.assertAlmostEqual(arc.radius,30.)
            self.assertAlmostEqual(arc.start_c,0.)
            self.assertAlmostEqual(arc.end_c,90.)
            self.assertAlmostEqual(arc.required_c_velocity_deg_s,math.degrees(40/30))
            self.assertEqual(sum(s.emit and s.generated_by_fillet for s in result.samples),1)
            self.assertTrue(any(s.display_only for s in result.samples))
            self.assertIn('I0.0000 J30.0000' if format=='CENTER_IJK' else 'R30.0000',result.gcode)
            outputs.append(arc)
        self.assertEqual([p.center_xyz for p in outputs],[outputs[0].center_xyz]*3)

    def test_square_four_fillets_and_feed_preserved(self):
        result=self.run_case(dict(fixtures())['square_R30'])
        self.assertEqual(len(self.arcs(result)),4)
        self.assertTrue(all(p.feed==2400 for p in self.arcs(result)))
        self.assertLess(len(result.final_motion_plan.primitives),15)

    def test_dense_circles_four_quarters_for_all_densities(self):
        for n in (360,720,2000):
            result=self.run_case(dict(fixtures())[f'circle_{n}'])
            self.assertEqual(len(self.arcs(result)),4)
            for arc in self.arcs(result):
                self.assertAlmostEqual(arc.sweep_rad,math.pi/2,delta=math.radians(.001))
                self.assertLessEqual(arc.metadata['max_fit_error_mm'],.30)
                self.assertLessEqual(arc.metadata['max_orientation_fit_error_deg'],1.)

    def test_ellipse_sine_spiral_bounded_and_continuous(self):
        for name in ('ellipse','sine','spiral'):
            result=self.run_case(dict(fixtures())[name])
            arcs=self.arcs(result)
            self.assertGreater(len(arcs),1)
            for arc in arcs:
                self.assertLessEqual(arc.metadata['max_fit_error_mm'],.30)
                self.assertLessEqual(arc.metadata['max_orientation_fit_error_deg'],1.+1e-8)
            for a,b in zip(arcs,arcs[1:]):
                if math.dist(a.end_xyz,b.start_xyz)<1e-8:
                    self.assertAlmostEqual(a.end_c,b.start_c,places=7)
                    delta=math.degrees(a.start_angle_rad+a.sweep_rad+math.copysign(math.pi/2,a.sweep_rad)-
                                       b.start_angle_rad-math.copysign(math.pi/2,b.sweep_rad))
                    self.assertLessEqual(abs((delta+180)%360-180),1.)
            if name=='sine': self.assertTrue(any(d['split_reason']=='INFLECTION' for d in result.report['native_arcs']['decisions']))
            if name=='spiral': self.assertGreater(arcs[-1].radius,arcs[0].radius)

    def test_center_modes_units_and_radius_signs(self):
        for direction in ('G2','G3'):
            for radius in (50,-50):
                motion=interpret(parse(f'G0 X50 Y0\n{direction} X0 Y50 R{radius} F600\n'))[0][-1]
                sweep=abs(arc_geometry(motion)[-1])
                self.assertLess(sweep,math.pi) if radius>0 else self.assertGreater(sweep,math.pi)
        a=interpret(parse('G0 X50 Y0\nG90.1\nG3 X0 Y50 I0 J0 F600\n'))[0][-1]
        b=interpret(parse('G0 X50 Y0\nG91.1\nG3 X0 Y50 I-50 J0 F600\n'))[0][-1]
        self.assertEqual(arc_geometry(a),arc_geometry(b))
        inch=interpret(parse('G20\nG0 X2 Y0\nG90.1\nG3 X0 Y2 I0 J0 F10\n'))[0][-1]
        self.assertAlmostEqual(arc_geometry(inch)[2],50.8)

    def test_full_circle_helix_unwrap_symmetry_and_P_turns(self):
        for symmetry in (180,360):
            for direction in ('G2','G3'):
                result=self.run_case(f'G0 X50 Y0 Z20\n{direction} X50 Y0 Z25 I-50 J0 P2 F2400\n',
                                     Config.from_dict({'orientation':{'symmetry_period_deg':symmetry}}))
                arcs=self.arcs(result)
                self.assertEqual(len(arcs),8)
                self.assertAlmostEqual(arcs[-1].end_c-arcs[0].start_c,720 if direction=='G3' else -720)
                self.assertAlmostEqual(sum(p.length_mm for p in arcs),math.hypot(100*math.pi*2,5))
                self.assertTrue(all(abs(p.end_c-p.start_c)<=90.00001 for p in arcs))

    def test_one_turn_full_circle_is_one_linuxcnc_ijk_block(self):
        for direction, expected in (("G2", -360.), ("G3", 360.)):
            result=self.run_case(f'G0 X50 Y0 Z20\n{direction} I-50 J0 F1200\n')
            arc,=self.arcs(result)
            self.assertAlmostEqual(math.degrees(arc.sweep_rad),expected,places=7)
            self.assertAlmostEqual(arc.end_c-arc.start_c,expected,places=7)
            blocks=[b for b in parse(result.gcode)
                    if any(k=='G' and v==int(direction[1]) for k,v in b.words)]
            self.assertEqual(len(blocks),1)
            self.assertIsNone(blocks[0].value('X')); self.assertIsNone(blocks[0].value('Y'))
            output,_=interpret(parse(result.gcode))
            exported=[m for m in output if m.kind.startswith('ARC')]
            self.assertEqual(len(exported),1)
            self.assertAlmostEqual(math.degrees(arc_geometry(exported[0])[-1]),expected,places=7)

    def test_full_circle_has_true_bounds_and_no_tiny_segment_warning(self):
        motions,_=interpret(parse('G0 X50 Y0 Z20\nG3 I-50 J0 F1200\n'))
        _,_,warnings=build(motions,Config())
        self.assertFalse(any('segmento XY pequeño' in warning for warning in warnings))
        points=[point for motion in motions for point in motion_extent_points(motion)]
        self.assertEqual((min(p[0] for p in points),max(p[0] for p in points)),(-50.,50.))
        self.assertEqual((min(p[1] for p in points),max(p[1] for p in points)),(-50.,50.))

    def test_wrap_179_181(self):
        angle=math.radians(89)
        result=self.run_case(f'G0 X{50*math.cos(angle):.9f} Y{50*math.sin(angle):.9f} Z20\nG3 X{50*math.cos(math.radians(91)):.9f} Y{50*math.sin(math.radians(91)):.9f} I{-50*math.cos(angle):.9f} J{-50*math.sin(angle):.9f} F600\n',
                             Config.from_dict({'orientation':{'alignment_offset_deg':0}}))
        arc,=self.arcs(result)
        self.assertAlmostEqual(arc.end_c-arc.start_c,2.,places=6)

    def test_Z_only_and_layers(self):
        result=self.run_case(dict(fixtures())['multilayer_square'])
        primitives=result.final_motion_plan.primitives
        for p in primitives:
            if p.type=='Z_ONLY':
                self.assertEqual(p.start_xyz[:2],p.end_xyz[:2]); self.assertEqual(p.start_c,p.end_c)
        self.assertEqual({p.layer_id for p in self.arcs(result)},{1,2})
        self.assertEqual(result.report['geometry']['max_interlayer_xy_drift_mm'],0.)

    def test_feed_process_and_locked_events_not_fitted_across(self):
        pts=[(50*math.cos(i*.01),50*math.sin(i*.01)) for i in range(101)]
        source=path(pts)
        lines=source.splitlines(); lines[34]+=' ; LOCKED'; lines[64]=lines[64].replace('F2400','F1800'); lines.insert(80,'M7 P2')
        result=self.run_case('\n'.join(lines),Config.from_dict({'serialization':{'motion_only':False}}))
        self.assertEqual(result.gcode.count('M7 P2'),1)
        protected={35,65,81}
        for p in self.arcs(result):
            covered=p.metadata['source_lines']
            self.assertFalse(any(min(covered)<line<max(covered) for line in protected))

    def test_fallback_preserve_mode_and_unsafe_precision(self):
        result=self.run_case(dict(fixtures())['sine'],Config.from_dict({'geometry':{'mode':'PRESERVE_PATH'}}))
        self.assertFalse(self.arcs(result))
        self.assertGreater(result.report['native_arcs']['fallback_G1_count'],0)
        result=self.run_case(dict(fixtures())['fillet_R30'],Config.from_dict({'nozzle':{'length_mm':30}}))
        arc=self.arcs(result)[0]
        almost_half=replace(arc,sweep_rad=math.pi,end_xyz=(270.,60.,20.))
        self.assertTrue(arc_words(almost_half,Config()).startswith('I'))
        bad=replace(arc,center_xyz=(270.233,30.111,20.),start_xyz=(270.233,.111,20.),end_xyz=(300.233,30.111,20.))
        config=Config.from_dict({'arc_output':{'format':'CENTER_IJK'},'serialization':{'xyz_decimals':0}})
        with self.assertRaises(ValueError): arc_words(bad,config)

    def test_kinematics_limit_warns_preserves_F(self):
        result=self.run_case(dict(fixtures())['fillet_R30'],Config.from_dict({'kinematics':{'max_c_velocity_deg_s':20},'corner_smoothing':{'radius_mode':'fixed','radius_mm':30}}))
        self.assertTrue(any('ARC_C_VELOCITY_LIMIT' in w for w in result.report['warnings']))
        self.assertTrue(all(p.feed==2400 for p in self.arcs(result)))

    def test_biarc_exact_tangent_and_degenerate_guard(self):
        arcs=biarc_hermite((0.,0.,20.),(30.,20.,20.),0.,65.)
        self.assertIsNotNone(arcs)
        a,b=arcs
        self.assertAlmostEqual(a.heading,0.,places=7)
        self.assertAlmostEqual(b.heading+math.degrees(b.sweep),65.,places=7)
        self.assertAlmostEqual(a.heading+math.degrees(a.sweep),b.heading,places=7)
        self.assertIsNone(biarc_hermite((0,0,0),(0,0,0),0,90))
        self.assertIsNone(circle_through((1e6,0),(1e6+1,0),(1e6+2,0)))

    def test_visual_backend_poses_fixture(self):
        result=self.run_case(dict(fixtures())['fillet_R30'])
        arc=self.arcs(result)[0]
        fixture={'primitive':arc.to_dict(),'expected':[{'t':t,'pose':arc.pose(t),'tangent_xy':arc.tangent_xy(t)} for t in (0.,.25,.5,.75,1.)]}
        Path('tests/native_arc_pose_fixture.json').write_text(json.dumps(fixture,indent=2),encoding='utf-8')

    def test_origin_policies_and_g1_comparison(self):
        source=dict(fixtures())['fillet_R30']
        for config in ({'native_arcs':{'enabled':False}}, {'native_arcs':{'corner_fillets':{'enabled':False}}},
                       {'serialization':{'motion_mode':'G1_ONLY'}}):
            config['nozzle']={'length_mm':30}
            result=self.run_case(source,Config.from_dict(config))
            self.assertFalse(self.arcs(result)); self.assertIn('G1 X300.000 Y30.000',result.gcode)
        result=self.run_case(source,Config.from_dict({'nozzle':{'length_mm':30},'native_arcs':{'corner_fillets':{'format':'CENTER_IJK'}}}))
        self.assertIn('I0.0000 J30.0000',result.gcode)

    def test_C_input_is_recorded_and_C_only_preserved(self):
        motions,_=interpret(parse('G0 X50 Y0 C100\nG91 G1 C20 F600\nG90\nG3 X0 Y50 I-50 J0 C210 F600\n'))
        self.assertEqual([(m.c_start,m.c_end) for m in motions],[(0,100),(100,120),(120,210)])
        result=self.run_case('G1 C30 F600\nG0 X50 Y0 Z20\nG3 X0 Y50 I-50 J0 F600\n')
        self.assertIn('G1 C30.000 F600.000',result.gcode)
        self.assertEqual(result.report['validators']['NativeArcs']['arc_count'],1)

    def test_native_serialization_selected_decimals(self):
        for decimals in (3,4,6):
            result=self.run_case(dict(fixtures())['fillet_R30'],Config.from_dict({'nozzle':{'length_mm':30},'serialization':{
                'xyz_decimals':decimals,'c_decimals':decimals,'ijk_decimals':decimals+1,'radius_decimals':decimals+1}}))
            self.assertIn('R'+f'{30.:.{decimals+1}f}',result.gcode)
            self.assertEqual(result.report['validators']['Serialization']['status'],'PASS')

    def test_invalid_geometry_and_config_are_rejected(self):
        for source in ('G5 X10 Y10 I0 J3 P0 Q-3 F600','G3 X10 Y0 R1 F600','G18 G2 X20 Z10 I10 K0 F600','G90.1 G3 X0 Y50 I0 F600',
                       'G3 X10 Y0 I0 J0 K10 F600'):
            with self.assertRaises(ValueError): convert(source)
        for data in ({'arc_fit':{'min_radius_mm':0}}, {'arc_output':{'format':'SPLINE'}},
                     {'native_arcs':{'source_arcs':{'format':'bad'}}}):
            with self.assertRaises(ValueError): Config.from_dict(data)

    def test_travel_arcs_are_never_replaced_by_single_chord(self):
        source='G0 X50 Y0 Z20\nM5\nG3 X50 Y0 I-50 J0 F600\n'
        result=self.run_case(source)
        self.assertEqual(len(self.arcs(result)),1)
        self.assertTrue(all(p.process_state=='TRAVEL' for p in self.arcs(result)))
        self.assertAlmostEqual(sum(p.length_mm for p in self.arcs(result)),100*math.pi)
        fallback=self.run_case(source,Config.from_dict({'native_arcs':{'enabled':False}}))
        self.assertFalse(self.arcs(fallback))
        self.assertGreater(len(fallback.final_motion_plan.primitives),20)

    def test_merge_revalidates_reference_and_respects_lock(self):
        points=[(50*math.cos(math.radians(i)),50*math.sin(math.radians(i))) for i in range(61)]
        motions,_=interpret(parse(path(points)))
        strokes,_,_=build(motions,Config())
        source=next(s for s in strokes if s.deposition)
        reference=Field(source,Config()); fitter=ArcFitter(reference,Config())
        start=Point((*points[0],20),4,1,2400,heading_override_deg=90.)
        mid=Point((*points[30],20),34,31,2400,arc_center=(0.,0.),arc_radius=50.,arc_sweep_deg=30.,
                  arc_source='FREE_CURVE_FIT',source_lines=list(range(4,35)),
                  fit_metadata={'source_span':'test','reference_s_start':0.,'reference_s_end':reference.s[30]})
        end=Point((*points[-1],20),64,61,2400,arc_center=(0.,0.),arc_radius=50.,arc_sweep_deg=30.,
                  arc_source='FREE_CURVE_FIT',source_lines=list(range(35,65)),
                  fit_metadata={'source_span':'test','reference_s_start':reference.s[30],'reference_s_end':reference.s[-1]})
        stroke=Stroke(source.id,source.layer_id,True,[start,mid,end])
        merged=merge_fitted_arcs(stroke,fitter,[])
        self.assertEqual(len(merged.points),2)
        self.assertAlmostEqual(merged.points[-1].arc_sweep_deg,60.)
        self.assertEqual(len(merge_fitted_arcs(stroke,fitter,[reference.s[30]]).points),3)

    def test_near_straight_curve_does_not_emit_enormous_radius(self):
        result=self.run_case(path([(i,.000001*i*i) for i in range(101)]))
        self.assertFalse(self.arcs(result))


if __name__=='__main__': unittest.main()
