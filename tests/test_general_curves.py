"""General planar contour regression and independent certificate checks."""
import math
import re
from pathlib import Path
from types import SimpleNamespace
import unittest
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.arc_fitting import FittedArc,fit_stroke
from backend.core.curve_reconstruction import ReconstructionFitter,reconstruct_stroke
from backend.core.fillet import optimize_stroke
from backend.core.gcode import parse,interpret
from backend.core.orientation import Field,wrap180
from backend.core.parametric import ParametricReference,BezierPiece
from backend.core.path import build
from backend.core.validators import ReconstructionValidator,NativeArcValidator
from backend.core.simplification import distance_to_chord
from tests.general_curve_fixtures import cases,organic_points
from tests.native_arc_benchmarks import path


def config(extra=None):
    result={'arc_fit':{'reference_model':'PARAMETRIC_HERMITE'},
            'serialization':{'xyz_decimals':6,'c_decimals':6,'ijk_decimals':6}}
    for section,values in (extra or {}).items(): result.setdefault(section,{}).update(values)
    return Config.from_dict(result)


class GeneralCurveTests(unittest.TestCase):
    def checked(self,source,cfg=None):
        result=convert(source,cfg or config())
        self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
        self.assertEqual(result.report['validators']['NativeArcs']['status'],'PASS')
        return result

    def test_sparse_organic_arcs_cover_all_four_regions(self):
        source=dict(cases())['organic_60_ccw']
        old=self.checked(source,Config()); new=self.checked(source)
        self.assertGreater(new.report['native_arcs']['fitted_arc_count'],old.report['native_arcs']['fitted_arc_count'])
        coverage=new.report['native_arcs']['coverage_by_stroke'][0]
        self.assertGreater(coverage['arc_length_fraction'],.70)
        self.assertTrue(all(x>.50 for x in coverage['arc_fraction_by_quarter']))
        self.assertTrue(new.report['curve_reconstruction']['applied'])
        self.assertEqual(new.report['curve_reconstruction']['orientation_reference'],'PARAMETRIC_TANGENT')
        self.assertFalse(new.report['curve_reconstruction']['source_function_recovered'])

    def test_closed_organic_both_directions_periodic_c(self):
        for key,sign in (('organic_180_ccw',1),('organic_180_cw',-1)):
            result=self.checked(dict(cases())[key])
            motions=[p for p in result.final_motion_plan.primitives if p.process_state=='DEPOSIT']
            self.assertLess(math.dist(motions[0].start_xyz,motions[-1].end_xyz),1e-7)
            self.assertAlmostEqual(motions[-1].end_c-motions[0].start_c,sign*360.,delta=1e-6)
            self.assertGreater(result.report['native_arcs']['native_arc_length_fraction'],.85)

    def test_figure_eight_has_zero_winding_and_no_duplicate_cut_failure(self):
        result=self.checked(dict(cases())['figure_eight'])
        self.assertGreater(result.report['native_arcs']['fitted_arc_count'],10)
        printed=[p for p in result.final_motion_plan.primitives if p.process_state=='DEPOSIT']
        self.assertAlmostEqual(printed[-1].end_c,printed[0].start_c,delta=1e-6)
        self.assertTrue(all(p.length_mm>1e-8 for p in printed))
        self.assertFalse(any(d['output']=='G1_FALLBACK' for d in result.report['native_arcs']['decisions']))

    def test_lobes_open_and_nonuniform_are_certified(self):
        fixtures=dict(cases())
        for key in ('four_lobes_120','four_lobes_360','open_variable_curve','nonuniform_organic'):
            result=self.checked(fixtures[key]); q=result.report['curve_reconstruction']
            self.assertTrue(q['applied'])
            self.assertLessEqual(q['max_certified_geometry_error_mm'],.30+1e-8)
            self.assertLessEqual(q['max_certified_orientation_error_deg'],1.+1e-8)
            self.assertEqual(result.report['validators']['CurveReconstruction']['status'],'PASS')

    def test_independent_radial_geometry_tangent_and_on_curve_join(self):
        cfg=config(); raw=next(s for s in build(interpret(parse(path(organic_points(180))))[0],cfg)[0] if s.deposition)
        stroke,_=optimize_stroke(raw,cfg); field=Field(stroke,cfg)
        reference=ParametricReference(stroke,field,0,len(stroke.points)-1,.15)
        fitter=ReconstructionFitter(reference,cfg)
        # Select a nontrivial inflection-free portion of the actual closed curve.
        cuts=reference.cuts(); a,b=max(zip(cuts,cuts[1:]),key=lambda pair:pair[1]-pair[0])
        result=fitter.fit(a,b)
        self.assertTrue(any(p.arc_center is not None for p in result[1:]))
        for start,end in zip(result,result[1:]):
            meta=end.fit_metadata; ua=meta['reference_u_start']; ub=meta['reference_u_end']
            self.assertLess(math.dist(start.xyz,reference.xyz(ua)),1e-7)
            self.assertLess(math.dist(end.xyz,reference.xyz(ub)),1e-7)
            for k in range(101):
                u=ua+(ub-ua)*k/100; q=reference.xyz(u)
                index,t=reference.location(u); piece=reference.pieces[index]
                source=stroke.points[piece.source_index:piece.source_index+2]
                source_error=distance_to_chord(q,source[0].xyz,source[1].xyz)
                self.assertLessEqual(source_error,piece.source_error_bound+1e-7)
                if end.arc_center is not None:
                    radius=math.dist(q[:2],end.arc_center)
                    radial_error=abs(radius-end.arc_radius)
                    phi=math.degrees(math.atan2(q[1]-end.arc_center[1],q[0]-end.arc_center[0]))
                    heading=phi+math.copysign(90,end.arc_sweep_deg)
                    angular_error=abs(wrap180(reference.heading(u)-heading))
                    self.assertLessEqual(radial_error,meta['approximation_error_mm']+1e-7)
                else:
                    chord=tuple(end.xyz[j]-start.xyz[j] for j in range(2)); norm=sum(v*v for v in chord)
                    t=sum((q[j]-start.xyz[j])*chord[j] for j in range(2))/norm
                    heading=reference.heading(ua)+(reference.heading(ub)-reference.heading(ua))*t
                    angular_error=abs(wrap180(reference.heading(u)-heading))
                    self.assertLessEqual(distance_to_chord(q,start.xyz,end.xyz),meta['approximation_error_mm']+1e-7)
                self.assertLessEqual(angular_error,meta['max_orientation_fit_error_deg']+1e-7)

    def test_radial_certificate_rejects_hidden_bezier_bulge(self):
        piece=BezierPiece(((50.,0.,20.),(50.,100.,20.),(100.,50.,20.),(0.,50.,20.)),0.,1.,90.,0.,0)
        reference=SimpleNamespace(source_error_bound=0.,heading=piece.heading,
                                  portions=lambda a,b:iter([(piece,piece.controls,a,b)]))
        fitter=ReconstructionFitter(reference,config())
        candidate=FittedArc(piece.controls[0],piece.controls[-1],(0.,0.),50.,math.pi/2,0.,1.)
        self.assertEqual(math.dist(candidate.start[:2],candidate.center),50.)
        self.assertEqual(math.dist(candidate.end[:2],candidate.center),50.)
        self.assertFalse(fitter.validate_arc(candidate)[0])

    def test_radius_limit_uses_certified_g1_without_losing_curve(self):
        result=self.checked(path(organic_points(60)),config({'arc_fit':{'min_radius_mm':1e6}}))
        q=result.report['curve_reconstruction']
        self.assertTrue(q['applied']); self.assertEqual(q['native_arc_count'],0)
        self.assertGreater(q['line_count'],10)
        self.assertLessEqual(q['max_certified_orientation_error_deg'],1.+1e-8)

    def test_preserve_and_disabled_options_keep_previous_mode(self):
        source=path(organic_points(60))
        for extra in ({'geometry':{'mode':'PRESERVE_PATH'}},{'arc_fit':{'enabled':False}},
                      {'native_arcs':{'enabled':False}},{'serialization':{'motion_mode':'G1_ONLY'}}):
            result=self.checked(source,config(extra))
            self.assertFalse(result.report['curve_reconstruction']['applied'])

    def test_source_arc_helix_and_sharp_fillets_still_preserved(self):
        result=self.checked('G0 X50 Y0 Z20\nG3 X50 Y0 Z25 I-50 J0 F2400\n')
        arcs=[p for p in result.final_motion_plan.primitives if p.type=='ARC']
        self.assertEqual(len(arcs),1)
        self.assertAlmostEqual(math.degrees(arcs[0].sweep_rad),360.,places=7)
        self.assertTrue(all(p.arc_source=='SOURCE_G2G3' for p in arcs))
        square=self.checked(path([(150,0),(300,0),(300,300),(0,300),(0,0),(150,0)]))
        self.assertEqual(square.report['native_arcs']['fillet_arc_count'],4)

    def test_locks_feeds_process_and_e_are_not_reconstructed_across(self):
        lines=path(organic_points(120)).splitlines()
        lines[30]+=' ; LOCKED'
        lines[60]=lines[60].replace('F2400','F1800')
        lines.insert(90,'M7 P2')
        result=self.checked('\n'.join(lines),config({'serialization':{'motion_only':False}}))
        self.assertEqual(result.gcode.count('M7 P2'),1)
        for p in result.final_motion_plan.primitives:
            if p.type!='ARC': continue
            covered=p.metadata['source_lines']
            self.assertFalse(any(min(covered)<k<max(covered) for k in (31,61,91)))
        e=self.checked('G0 X0 Y0 Z20\nM3\nG1 X10 Y1 E1 F600\nG1 X20 Y3 E2\nG1 X30 Y6 E3\nM5\n',
                       config({'geometry':{'mode':'PRESERVE_PATH'}}))
        self.assertFalse(e.report['curve_reconstruction']['applied'])

    def test_multiple_layers_unchanged_z_and_c_motion(self):
        result=self.checked(dict(cases())['organic_two_layers'])
        self.assertEqual(result.report['z_conversion']['target_layer_z_values'],[20.,38.])
        self.assertEqual(result.report['geometry']['max_interlayer_xy_drift_mm'],0.)
        self.assertTrue(all(p.feed==2400 for p in result.final_motion_plan.primitives if p.process_state=='DEPOSIT'))

    def test_translation_and_symmetry(self):
        source=path([(x+1e6,y-2e6) for x,y in organic_points(60)])
        result=self.checked(source,config({'orientation':{'symmetry_period_deg':180}}))
        self.assertTrue(result.report['curve_reconstruction']['applied'])
        self.assertGreater(result.report['native_arcs']['native_arc_length_fraction'],.5)

    def test_user_wall_arcs_extend_past_first_line_every_layer(self):
        source=Path('pared_senoidal_concreto_3d.gcode').read_text(encoding='utf-8-sig')
        result=self.checked(source)
        self.assertEqual(len(result.report['layers']),10)
        self.assertGreater(result.report['native_arcs']['native_arc_length_fraction'],.9)
        rows=result.report['native_arcs']['coverage_by_stroke']
        self.assertEqual(len(rows),10)
        self.assertTrue(all(all(v>.8 for v in row['arc_fraction_by_quarter']) for row in rows))
        self.assertEqual(result.report['z_conversion']['target_layer_z_values'],[20+18*i for i in range(10)])

    def test_invalid_configuration_and_tampered_certificate_rejected(self):
        for extra in ({'reference_model':'GUESS'},{'reconstruction_budget_fraction':0},
                      {'reconstruction_budget_fraction':1},{'reconstruction_budget_fraction':True}):
            with self.assertRaises(ValueError): config({'arc_fit':extra})
        result=self.checked(path(organic_points(180)))
        primitive=next(p for p in result.final_motion_plan.primitives if p.metadata.get('reference_model')=='PARAMETRIC_HERMITE')
        primitive.metadata['max_fit_error_mm']=.31
        self.assertEqual(ReconstructionValidator.check(result.final_motion_plan.primitives,config())['status'],'FAIL')

    def test_export_pose_audit_detects_damage_from_coarse_rounding(self):
        result=self.checked(path(organic_points(60)))
        coarse=re.sub(r'\b([XYZCIJ])(-?\d+(?:\.\d+)?)',
                      lambda m:m[1]+f'{float(m[2]):.3f}',result.gcode)
        audit=NativeArcValidator.check(coarse,result.final_motion_plan.primitives,
                                      config({'serialization':{'xyz_decimals':3,'c_decimals':3,'ijk_decimals':3}}))
        self.assertEqual(audit['status'],'FAIL')
        self.assertTrue(any('redondeado supera tolerancia' in x for x in audit['errors']))


if __name__=='__main__': unittest.main()
