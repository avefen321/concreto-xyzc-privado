import unittest
from dataclasses import replace
from backend.core.primitives import MotionPrimitive
from backend.core.motion_timing import estimate_time,trapezoid


class TimingTests(unittest.TestCase):
    def primitive(self,**extra):
        p=MotionPrimitive('P1',1,1,1,'LINE',(0.,0.,0.),(100.,0.,0.),0.,0.,600.,'DEPOSIT',1,1)
        return replace(p,**extra)

    def test_triangular_trapezoid_and_missing_configuration(self):
        self.assertAlmostEqual(trapezoid(100,10,10)['seconds'],11.)
        self.assertAlmostEqual(trapezoid(1,10,1)['seconds'],2.)
        self.assertEqual(estimate_time([self.primitive()])['status'],'NOT_CONFIGURED')
        limits={k:{'velocity':10.,'acceleration':10.} for k in 'XYZC'}
        r=estimate_time([self.primitive()],limits,policy='EXACT_STOP')
        self.assertAlmostEqual(r['t_sim_s'],11.); self.assertIsNone(r['t_real_s'])
        c=self.primitive(type='C_ONLY',start_xyz=(0.,0.,0.),end_xyz=(0.,0.,0.),end_c=100.)
        self.assertAlmostEqual(estimate_time([c],limits,policy='EXACT_STOP')['t_sim_s'],11.)

    def test_collinear_continuation_and_c_velocity_discontinuity(self):
        limits={k:{'velocity':10.,'acceleration':10.} for k in 'XYZC'}
        a=self.primitive(end_xyz=(50.,0.,0.))
        b=self.primitive(id='P2',start_xyz=(50.,0.,0.))
        self.assertAlmostEqual(estimate_time([a,b],limits,policy='CONTINUOUS_TANGENT')['t_sim_s'],11.)
        self.assertAlmostEqual(estimate_time([a,b],limits,policy='EXACT_STOP')['t_sim_s'],12.)
        r=estimate_time([a,replace(b,end_c=90.)],limits,policy='CONTINUOUS_TANGENT')
        self.assertEqual(r['segments'][0]['exit_velocity'],0.)
