import importlib.util
import math
import unittest
from backend.core.parametric import BezierPiece,coefficients,polynomial_range
from backend.core.nozzle_footprint import rectangle


class OracleTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec('scipy'),'Extra científico opcional')
    def test_bezier_values_and_derivatives_against_scipy_bernstein(self):
        import numpy as np
        from scipy.interpolate import BPoly
        controls=((0.,0.,20.),(20.,30.,20.),(40.,-10.,20.),(60.,0.,20.))
        piece=BezierPiece(controls,0.,1.,0.,.1,0)
        oracle=BPoly(np.asarray(controls)[:,None,:],[0.,1.])
        for u in np.linspace(0,1,23):
            self.assertLess(math.dist(piece.xyz(float(u)),oracle(u)),1e-10)
            self.assertLess(math.dist(piece.tangent(float(u)),oracle.derivative()(u)[:2]),1e-10)

    def test_rectangle_rotations_and_translation(self):
        for angle in (0,45,90,180,359):
            q=rectangle(100,200,angle)
            self.assertAlmostEqual(math.dist(q[0],q[1]),35.)
            self.assertAlmostEqual(math.dist(q[1],q[2]),15.)
            self.assertAlmostEqual(sum(p[0] for p in q)/4,100.)
            self.assertAlmostEqual(sum(p[1] for p in q)/4,200.)

    def test_polynomial_extrema_contain_dense_independent_values(self):
        poly=[.2,-3.,5.,-2.]
        lo,hi=polynomial_range(poly)
        values=[sum(c*(i/1000)**j for j,c in enumerate(poly)) for i in range(1001)]
        self.assertLessEqual(lo,min(values));self.assertGreaterEqual(hi,max(values))
