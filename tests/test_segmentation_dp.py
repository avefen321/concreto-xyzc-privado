import itertools
import unittest
from backend.core.segmentation_optimizer import CandidatePrimitive,shortest_path,SearchParameters


class DPTests(unittest.TestCase):
    def test_optimum_matches_exhaustive_paths(self):
        for n in range(2,8):
            edges=[CandidatePrimitive(i,j,(),1+(j-i)%3,'TEST') for i in range(n) for j in range(i+1,n)]
            def costs(i):
                if i==n-1: return [0]
                return [e.cost+c for e in edges if e.start==i for c in costs(e.end)]
            self.assertEqual(sum(e.cost for e in shortest_path(n,edges)),min(costs(0)))

    def test_join_state_excludes_incompatible_cheaper_path(self):
        edges=[CandidatePrimitive(0,1,(),1,'BAD'),CandidatePrimitive(0,1,(),2,'GOOD'),
               CandidatePrimitive(1,2,(),1,'END')]
        path=shortest_path(3,edges,lambda a,b:a.kind!='BAD')
        self.assertEqual([p.kind for p in path],['GOOD','END'])
        self.assertIsNone(shortest_path(3,edges,lambda a,b:False))

    def test_invalid_parameters_and_edges(self):
        for p in ({'candidate_window':0},{'split_level':3},{'max_sweep_deg':float('nan')}):
            with self.assertRaises(ValueError): SearchParameters.from_dict(p)
        with self.assertRaises(ValueError): shortest_path(2,[CandidatePrimitive(1,0,(),1,'BAD')])
