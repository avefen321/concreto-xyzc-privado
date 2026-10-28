import unittest
from unittest.mock import patch
from backend.optimization.search import search,availability
from backend.optimization.statistics import wilson,paired_summary,reduction


class SearchTests(unittest.TestCase):
    def test_algorithms_reproduce_and_count_invalid_attempts(self):
        space={'x':(-1.,0.,1.),'y':(0,1,2)}
        objective=lambda p:{'objective':p['x']**2+p['y']**2 if p['y']<2 else 10.,'feasible':p['y']<2}
        for method in ('random','bo','tpe'):
            if not availability()[method]: continue
            a=search(objective,method,7,11,space=space,initial=3)
            b=search(objective,method,7,11,space=space,initial=3)
            self.assertEqual([r['parameters'] for r in a['history']],[r['parameters'] for r in b['history']])
            self.assertEqual(a['best'],b['best']); self.assertEqual(a['evaluations_attempted'],7)
            self.assertTrue(all(r['objective']<float('inf') for r in a['history']))

    def test_cancel_and_optional_dependencies(self):
        self.assertEqual(search(lambda p:None,'random',4,7,cancel=lambda:True)['stop_reason'],'CANCELLED')
        with patch('backend.optimization.search.availability',return_value={'random':True,'bo':False,'tpe':False}):
            with self.assertRaises(ValueError): search(lambda p:None,'bo')

    def test_statistics_are_not_accuracy_percentages(self):
        lo,hi=wilson(9,10)
        self.assertAlmostEqual(lo,.652,places=2); self.assertAlmostEqual(hi,.977,places=2)
        self.assertIsNone(wilson(0,0)); self.assertIsNone(reduction(0,0))
        self.assertEqual(reduction(100,125),-25)
        self.assertIsNone(paired_summary([0,0,0])['t_score'])
        self.assertEqual(paired_summary([1,2,3])['inference'],'DESCRIPTIVE_PILOT')
