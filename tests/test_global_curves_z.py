import math
import unittest
from pathlib import Path
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse, interpret
from backend.core.path import build
from backend.core.fillet import optimize_stroke
from tests.test_optimization import path


def layers(zs):
    text='G21 G90\nG0 X0 Y0 Z%g\n'%zs[0]
    for z in zs:
        text+=f'G0 Z{z}\nM3\nG1 X300 Y0 F2400\nG1 Y300\nG1 X0\nG1 Y0\nM5\n'
    return text


class GlobalCurveTests(unittest.TestCase):
    def check_pass(self,r):
        self.assertNotEqual(r.report['status'],'FAIL',r.report['errors'])
        self.assertTrue(r.gcode)
        self.assertEqual(r.report['validators']['Serialization']['status'],'PASS')
        self.assertEqual(r.report['validators']['Sampling']['status'],'PASS')

    def test_collinear_300_and_locked_feed_process(self):
        r=convert(path([(x,0) for x in range(301)]))
        self.check_pass(r)
        self.assertEqual(sum(s.emit and s.deposition_active for s in r.samples),1)
        r=convert('M3\nG1 X50 F600\nG1 X100 F800\nM7 P2\nG1 X150 F800\nS123\nG1 X300\n',
                  Config.from_dict({'serialization':{'motion_only':False}}))
        self.check_pass(r)
        self.assertEqual(r.gcode.count('M7 P2'),1)
        self.assertEqual(r.gcode.count('S123'),1)
        self.assertTrue(any(s.emit and s.X==50 and s.feed_mm_min==600 for s in r.samples))

    def test_analytic_fillet_until_sampler_and_r30_formula(self):
        source=path([(0,0),(300,0),(300,300)],feed=2400)
        # This historical R30 formula uses an explicit 30 mm profile.
        cfg=Config.from_dict({'nozzle':{'length_mm':30}}); motions,_=interpret(parse(source)); strokes,_l,_w=build(motions,cfg)
        stroke=next(s for s in strokes if s.deposition)
        optimized,fillets=optimize_stroke(stroke,cfg)
        self.assertEqual(len(optimized.points),4) # start, T1, T2, end
        self.assertEqual(sum(p.arc_center is not None for p in optimized.points),1)
        r=convert(source,cfg); self.check_pass(r); f=r.report['fillets'][0]
        self.assertAlmostEqual(f['radius_mm'],30)
        n=math.ceil((math.pi/2)/min(2*math.acos(1-.3/30),math.radians(15),10/30))
        self.assertEqual(f['sample_count'],n)
        self.assertAlmostEqual(f['max_chord_error_mm'],30*(1-math.cos(math.pi/2/n/2)))
        self.assertAlmostEqual(f['required_omega_C_deg_s'],math.degrees(40/30))
        self.assertLess(f['corner_orientation_error_at_arc_exit_deg'],1e-6)
        legacy=convert(source,Config.legacy()); self.check_pass(legacy)
        self.assertEqual(legacy.report['fillets'][0]['sample_count'],45)

    def test_fixed_r10_adaptive_is_not_two_degrees(self):
        cfg=Config.from_dict({'corner_smoothing':{'radius_mode':'fixed','radius_mm':10}})
        r=convert(path([(0,0),(100,0),(100,100)]),cfg);self.check_pass(r)
        self.assertEqual(r.report['fillets'][0]['sample_count'],6)

    def test_radius_limits_neighbors_and_omega(self):
        cfg=Config.from_dict({'kinematics':{'max_c_velocity_deg_s':20},'corner_smoothing':{'max_path_deviation_mm':2}})
        r=convert(path([(0,0),(25,0),(25,20),(50,20)],feed=2400),cfg);self.check_pass(r)
        a,b=r.report['fillets']
        self.assertLessEqual(a['t2'][1]+5,b['t1'][1]+1e-8)
        self.assertTrue(all(f['kinematic_status']=='CORNER_KINEMATIC_LIMIT' for f in (a,b)))
        self.assertTrue(all(f['max_polyline_deviation_mm']<=2+1e-8 for f in (a,b)))
        self.assertTrue(all(s.feed_mm_min==2400 for s in r.samples if s.deposition_active))

    def test_circle_density_independence(self):
        results=[]
        for n in (40,4000):
            pts=[(30*math.cos(i*math.pi/2/n),30*math.sin(i*math.pi/2/n)) for i in range(n+1)]
            r=convert(path(pts));self.check_pass(r)
            self.assertFalse(r.report['fillets']);self.assertFalse(r.report['corners'])
            self.assertLessEqual(r.report['simplification']['max_geometry_error_mm'],.3)
            self.assertLessEqual(r.report['simplification']['max_orientation_error_deg'],1)
            results.append(r)
        counts=[sum(s.emit and s.deposition_active for s in r.samples) for r in results]
        self.assertLess(max(counts)/min(counts),2)
        self.assertLess(abs(results[0].samples[-1].C_deg-results[1].samples[-1].C_deg),1)

    def test_varying_curvature_curves_and_refinement(self):
        for kind in ('ellipse','sine','s_curve','spiral'):
            pts=[]
            for i in range(301):
                t=i/300
                if kind=='ellipse': p=(100*math.cos(t*math.pi),30*math.sin(t*math.pi))
                elif kind=='sine': p=(100*t,12*math.sin(t*2*math.pi))
                elif kind=='s_curve': p=(100*t,30*(t-.5)**3)
                else:p=((30+20*t)*math.cos(t*4*math.pi),(30+20*t)*math.sin(t*4*math.pi))
                pts.append(p)
            r=convert(path(pts));self.check_pass(r)
            self.assertFalse(r.report['fillets']);self.assertFalse(r.report['corners'])
            self.assertLess(sum(s.emit and s.deposition_active for s in r.samples),300)
        coarse=path([(50*math.cos(t),50*math.sin(t)) for t in (0,.2,.4,.6,.8,1.)])
        cfg=Config.from_dict({'native_arcs':{'enabled':False},'sampling':{'max_delta_c_deg':3,'max_orientation_interp_error_deg':.05}})
        r=convert(coarse,cfg);self.check_pass(r)
        self.assertGreater(sum(s.emit and s.deposition_active for s in r.samples),5)

    def test_native_full_circle_and_helix(self):
        for direction in ('G2','G3'):
            for dz in (0,5):
                r=convert(f'G0 X30 Y0\nM3\n{direction} X30 Y0 Z{dz} I-30 J0 F600\n')
                self.check_pass(r)
                arc=[s for s in r.samples if s.primitive_type=='ARC']
                self.assertTrue(arc)
                self.assertAlmostEqual(arc[-1].C_deg-next(s for s in r.samples if s.motion_type=='START').C_deg,360 if direction=='G3' else -360)
                self.assertTrue(all(abs(math.hypot(s.X,s.Y)-30)<1e-7 for s in arc))
                self.assertAlmostEqual(arc[-1].Z,20+dz)

    def test_extreme_chord_tolerance_safe(self):
        cfg=Config.from_dict({'sampling':{'geometry_error_mm':100},'corner_smoothing':{'max_chord_error_mm':100}})
        r=convert('G0 X1\nM3\nG3 X0 Y1 I-1 J0 F60\n',cfg); self.check_pass(r)

    def test_native_preserve_path_uses_analytic_centroid(self):
        r=convert('G0 X10 Y0\nM3\nG3 X0 Y10 I-10 J0 F600\n',Config.from_dict({'geometry':{'mode':'PRESERVE_PATH'}}))
        self.check_pass(r)
        self.assertAlmostEqual(r.report['layers'][0]['path_length_mm'],5*math.pi)
        self.assertAlmostEqual(r.report['layers'][0]['original_centroid_xy'][0],20/math.pi)

    def test_independent_circle_error_and_orientation(self):
        r=convert(path([(30*math.cos(i*math.pi/800),30*math.sin(i*math.pi/800)) for i in range(401)]))
        pts=[s for s in r.samples if s.deposition_active]
        for a,b in zip(pts,pts[1:]):
            midpoint=((a.X+b.X)/2,(a.Y+b.Y)/2)
            self.assertLessEqual(30-math.hypot(*midpoint),.301)
            angle=math.degrees(math.atan2(midpoint[1],midpoint[0]))+180
            physical=(a.C_deg-a.c_reference_shift_deg+b.C_deg-b.c_reference_shift_deg)/2
            self.assertLess(abs((physical-angle+180)%360-180),1)

    def test_process_events_and_path_mode(self):
        cfg=Config.from_dict({'geometry':{'mode':'PRESERVE_PATH'},'serialization':{'motion_only':False}})
        r=convert('G1 X50 F600\nG1 E1\nG1 X100\nG61\nG1 X150\n',cfg)
        self.check_pass(r)
        self.assertLess(r.gcode.index('X50.000'),r.gcode.index('G1 E1.000'))
        self.assertLess(r.gcode.index('G1 E1.000'),r.gcode.index('X100.000'))
        self.assertTrue(any('EXACT_PATH_MODE' in w for w in r.report['warnings']))
        r=convert('G61\nG1 X100 F600\n',Config.from_dict({
            'linuxcnc':{'path_mode':'G64','blend_tolerance_mm':.2},
            'orientation':{'initial_orientation_mode':'HOME_ALIGNED_TANGENT'}}))
        self.assertIn('G64 P0.2',r.gcode);self.assertNotIn('\nG61',r.gcode)
        with self.assertRaisesRegex(ValueError,'G93/G95'):convert('G93\nG1 X100 F1\n')


class LayerScheduleTests(unittest.TestCase):
    def test_schedules_and_vertical_transitions(self):
        for zs in ([0,20,40,60],[20,40,60,80],[0,20,40,60,80,100]):
            r=convert(layers(zs),Config.from_dict({'layer_transition':{'safe_seam':True}}))
            self.assertNotEqual(r.report['status'],'FAIL',r.report['errors'])
            expected=[20+18*k for k in range(len(zs))]
            self.assertEqual(r.report['z_conversion']['target_layer_z_values'],expected)
            self.assertEqual(r.report['z_conversion']['max_layer_z_error_mm'],0)
            printed={s.Z for s in r.samples if s.deposition_active}
            self.assertEqual(printed,set(expected))
            transitions=r.simulation['layer_transitions']
            for t in transitions:
                if t['start_xyz']:
                    self.assertEqual(t['start_xyz'][:2],t['xyz'][:2])
                    if t['xyz'][2]>20:self.assertEqual(t['xyz'][2]-t['start_xyz'][2],18)
            self.assertEqual(r.gcode,convert(layers(zs),Config.from_dict({'layer_transition':{'safe_seam':True}})).gcode)

    def test_safe_travel_and_relative_clearance(self):
        source='G0 Z200 ; MACHINE_SAFE_Z\nG0 X0 Y0\nG0 Z10\nM3\nG1 X100 F600\nM5\nG0 Z15 ; LAYER_RELATIVE_Z\nG0 X200\nG0 Z10\nM3\nG1 X300\n'
        r=convert(source)
        self.assertNotEqual(r.report['status'],'FAIL',r.report['errors'])
        self.assertEqual(next(s for s in r.samples if s.source_line==1).Z,200)
        self.assertEqual(next(s for s in r.samples if s.source_line==7).Z,25)
        self.assertEqual(r.report['counts']['layers'],1)

    def test_z_does_not_change_xy_or_c(self):
        source=layers([0,20,40])
        a=convert(source);b=convert(source,Config.from_dict({'z_conversion':{'enabled':False}}))
        self.assertEqual([(s.X,s.Y,s.C_deg) for s in a.samples if s.deposition_active],
                         [(s.X,s.Y,s.C_deg) for s in b.samples if s.deposition_active])

    def test_user_files(self):
        for filename,expected in [('cruz_CA.gcode',30),('hexagono_suave_CA.gcode',30),('pared_senoidal_concreto_3d.gcode',10)]:
            source=Path(filename).read_text(encoding='utf-8-sig');r=convert(source)
            self.assertNotEqual(r.report['status'],'FAIL',(filename,r.report['errors']))
            self.assertEqual(r.report['counts']['layers'],expected)
            self.assertEqual(r.report['z_conversion']['max_layer_z_error_mm'],0)
            self.assertLess(sum(s.emit and s.deposition_active for s in r.samples),5000)


if __name__=='__main__':unittest.main()
