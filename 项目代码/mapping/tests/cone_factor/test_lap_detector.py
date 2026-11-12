import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from lap_detector import LapDetector

class LapDetectorTests(unittest.TestCase):
    def test_stopped_or_short_motion_never_triggers(self):
        detector=LapDetector()
        for i in range(50): self.assertFalse(detector.update(i,math.sin(i)*.1,0,0))

    def test_wrong_heading_does_not_trigger(self):
        detector=LapDetector(min_distance=10,depart_distance=3,return_radius=1,min_seconds=1)
        for i in range(21):
            x=i*.5 if i<=10 else (20-i)*.5
            self.assertFalse(detector.update(i,x,0,0 if i==0 else math.pi))

    def test_return_to_start_after_one_circle(self):
        detector=LapDetector(min_distance=20,depart_distance=5,return_radius=1.5,min_seconds=15)
        triggered=[]
        for i in range(101):
            a=2*math.pi*i/100
            if detector.update(i,6*math.sin(a),6*(1-math.cos(a)),a):triggered.append(i)
        self.assertEqual(len(triggered),1)

if __name__=='__main__': unittest.main()
