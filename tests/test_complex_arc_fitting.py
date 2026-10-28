"""Independent checks for variable biarcs, continuous bounds and mixed C(s)."""
import math
import unittest
from dataclasses import replace
from backend.core.arc_fitting import ArcFitter,FittedArc,biarc_hermite,fit_stroke,machine_block_cost
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse,interpret
from backend.core.orientation import Field,wrap180
from backend.core.path import Point,Stroke,build
from tests.native_arc_benchmarks import fixtures,path


def coarse_sine():
    return path([(i,50*math.sin(i*math.pi/100)) for i in range(0,171,10)])


def source_stroke(source,config):
    return next(s for s in build(interpret(parse(source))[0],config)[0] if s.deposition)


class ComplexArcFittingTests(unittest.TestCase):
    def test_variable_biarcs_preserve_tangents_under_large_translation(self):
        a=(1e8,-2e8,20.); b=(1e8+100,-2e8+40,20.)
        joins=[]
        for ratio in (.125,.5,1.,2.,8.):
            arcs=biarc_hermite(a,b,0.,65.,ratio)
            self.assertIsNotNone(arcs)
            first,second=arcs; joins.append(first.end)
            self.assertAlmostEqual(wrap180(first.heading),0.,delta=1e-6)
            self.assertAlmostEqual(wrap180(second.heading+math.degrees(second.sweep)),65.,delta=1e-6)
            self.assertAlmostEqual(wrap180(first.heading+math.degrees(first.sweep)-second.heading),0.,delta=1e-6)
            self.assertLess(math.dist(first.xyz(1),second.xyz(0)),1e-6)
            self.assertEqual(second.end,b)
        self.assertGreater(math.dist(joins[0],joins[-1]),10.)
        for ratio in (0.,-1.,float('inf'),float('nan')):
            self.assertIsNone(biarc_hermite(a,b,0.,65.,ratio))
        self.assertIsNone(biarc_hermite((0,0,0),(100,0,0),0,0))

    def circle_reference(self,tolerance):
        config=Config.from_dict({'arc_fit':{'geometry_tolerance_mm':tolerance,'radial_tolerance_mm':tolerance},
                                'sampling':{'geometry_error_mm':tolerance},
                                'simplification':{'geometry_tolerance_mm':tolerance}})
        points=[Point((20*math.cos(t),20*math.sin(t),20),i+1,i,1200,
                      heading_override_deg=90+math.degrees(t)) for i,t in enumerate((0.,.2,.4))]
        field=Field(Stroke(1,1,True,points),config)
        arc=FittedArc(points[0].xyz,points[-1].xyz,(0.,0.),20.,.4,0.,field.s[-1])
        return ArcFitter(field,config),arc

    def test_refined_certificate_accepts_valid_coarse_chords(self):
        fitter,arc=self.circle_reference(.09995)
        self.assertGreater(20*.2**2/8,fitter.geometry_tolerance)
        good,reason,_=fitter.validate(arc)
        self.assertTrue(good,reason)
        measured=0.
        for i in range(1001):
            t=i/1000; s=arc.sb*t
            measured=max(measured,math.dist(fitter.reference_xyz(s),arc.xyz(t)))
        self.assertLessEqual(measured,arc.metrics['max_fit_error_mm']+1e-10)
        self.assertLessEqual(arc.metrics['max_fit_error_mm'],.09995)

    def test_certificate_rejects_between_vertex_error(self):
        fitter,arc=self.circle_reference(.0998)
        self.assertLess(math.dist(fitter.reference_xyz(arc.sb/2),arc.xyz(.5)),1e-10)
        good,reason,_=fitter.validate(arc)
        self.assertFalse(good)
        self.assertEqual(reason,'FIT_ERROR')
        self.assertGreater(arc.metrics['max_fit_error_mm'],.0998)

    def test_coarse_sine_arcs_and_remaining_lines_keep_reference_c(self):
        config=Config(); stroke=source_stroke(coarse_sine(),config)
        reference=Field(stroke,config); fitter=ArcFitter(reference,config)
        fitted,_=fit_stroke(stroke,config); final=Field(fitted,config)
        self.assertTrue(any(p.arc_source=='FREE_CURVE_FIT' for p in fitted.points))
        self.assertTrue(any('reference_orientation_pieces' in p.fit_metadata for p in fitted.points))
        for k,p in enumerate(fitted.points[1:]):
            if p.arc_source=='FREE_CURVE_FIT':
                sa=p.fit_metadata['reference_s_start']; sb=p.fit_metadata['reference_s_end']
                for j in range(101):
                    t=j/100; s=sa+(sb-sa)*t
                    self.assertLessEqual(math.dist(final.xyz_at(k,t),fitter.reference_xyz(s)),.30+1e-8)
                    self.assertLessEqual(abs(final.at(k,t)-fitter.reference_c(s)),1.+1e-8)
            elif 'reference_orientation_pieces' in p.fit_metadata:
                sa=p.fit_metadata['reference_s_start']; sb=p.fit_metadata['reference_s_end']
                for t in (0.,.1,.25,.5,.75,.9,1.):
                    self.assertAlmostEqual(final.at(k,t),fitter.reference_c(sa+(sb-sa)*t),delta=1e-8)

    def test_virtual_refinement_reduces_cost_when_useful(self):
        config=Config.from_dict({'sampling':{'max_linear_step_mm':1.}})
        stroke=source_stroke(coarse_sine(),config)
        refined,logs=fit_stroke(stroke,config)
        coarse_config=replace(config,arc_fit=replace(config.arc_fit,adaptive_refinement=False))
        coarse,_=fit_stroke(stroke,coarse_config)
        self.assertTrue(any(d['output']=='ADAPTIVE_REFINEMENT' for d in logs))
        self.assertGreater(sum(p.arc_source=='FREE_CURVE_FIT' for p in refined.points),
                           sum(p.arc_source=='FREE_CURVE_FIT' for p in coarse.points))
        self.assertLess(machine_block_cost(Field(refined,config),config),
                        machine_block_cost(Field(coarse,config),config))
        result=convert(coarse_sine(),config)
        self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
        self.assertEqual(result.report['validators']['NativeArcs']['status'],'PASS')

    def test_cost_guard_and_variable_ratios_on_real_fixtures(self):
        result=convert(dict(fixtures())['ellipse'])
        arcs=[p for p in result.final_motion_plan.primitives if p.type=='ARC']
        self.assertTrue(any(p.metadata.get('biarc_ratio',1.)!=1. for p in arcs))
        self.assertLessEqual(result.report['native_arcs']['max_orientation_fit_error_deg'],1.+1e-8)
        config=Config(); fitted,logs=fit_stroke(source_stroke(coarse_sine(),config),config)
        for log in logs:
            if log['output'] in ('REFINEMENT_REJECTED','COARSE_FIT_SELECTED'):
                self.assertGreaterEqual(log['refined_machine_blocks'],log['coarse_machine_blocks'])
        result=convert(coarse_sine())
        arcs=[p for p in result.final_motion_plan.primitives if p.type=='ARC']
        self.assertTrue(arcs)
        self.assertTrue(all(' I' in line and ' J' in line for line in result.gcode.splitlines()
                            if line.startswith(('G2 ','G3 '))))

    def test_refinement_settings_and_strict_geometry(self):
        for params in ({'adaptive_refinement':1},{'max_refinement_depth':-1},
                       {'max_refinement_depth':6},{'max_refinement_depth':1.5}):
            with self.assertRaises(ValueError): Config.from_dict({'arc_fit':params})
        result=convert(coarse_sine(),Config.from_dict({'geometry':{'mode':'PRESERVE_PATH'}}))
        self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
        self.assertFalse(any(p.type=='ARC' and p.arc_source=='FREE_CURVE_FIT'
                             for p in result.final_motion_plan.primitives))

    def test_refinement_preserves_lock_feed_and_process_boundaries(self):
        lines=coarse_sine().splitlines()
        lines[8]+=' ; LOCKED'
        lines[12]=lines[12].replace('F2400','F1800')
        lines.insert(15,'M7 P2')
        config=Config.from_dict({'sampling':{'max_linear_step_mm':1.},
                                'serialization':{'motion_only':False}})
        result=convert('\n'.join(lines),config)
        self.assertNotEqual(result.report['status'],'FAIL',result.report['errors'])
        self.assertEqual(result.gcode.count('M7 P2'),1)
        for arc in (p for p in result.final_motion_plan.primitives if p.type=='ARC'):
            covered=arc.metadata['source_lines']
            self.assertFalse(any(min(covered)<line<max(covered) for line in (9,13,16)))


if __name__=='__main__': unittest.main()
