import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from vote_classification import classify_votes

class VoteTests(unittest.TestCase):
    def test_preserve_repeated_hits_and_filter_weak_hit(self):
        hit=np.array([2,1,1,0,0]); passes=np.array([20,20,3,1,5])
        self.assertEqual(classify_votes(hit,passes).tolist(),[100,0,100,-1,0])

    def test_strict_ratio_rejects_repeated_weak_hits(self):
        hit=np.array([2,1,1,0,0]); passes=np.array([20,2,3,1,0])
        self.assertEqual(classify_votes(hit,passes,.35,3,True).tolist(),[0,100,0,0,-1])

    def test_middle_threshold_keeps_weaker_observations(self):
        hit=np.array([3,2]); passes=np.array([10,10])
        self.assertEqual(classify_votes(hit,passes,.275,3,True).tolist(),[100,0])
        self.assertEqual(classify_votes(hit,passes,.35,3,True).tolist(),[0,0])

if __name__=='__main__':unittest.main()
