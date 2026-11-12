import sys
import unittest
from pathlib import Path
SCRIPTS=Path(__file__).resolve().parents[2]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from cone_factor_geom import Pose, ConsecutiveCorrectionGate


class CorrectionGateTests(unittest.TestCase):
    def test_two_frames_same_odometry_correction_accept(self):
        gate=ConsecutiveCorrectionGate(.35,.16)
        self.assertFalse(gate.consider(Pose(-3,2,.3),15))
        self.assertTrue(gate.consider(Pose(-2.9,2.08,.34),16))
        self.assertIsNone(gate.pending)

    def test_no_single_frame_false_positive_or_spaced_gaps(self):
        gate=ConsecutiveCorrectionGate(.35,.16)
        self.assertFalse(gate.consider(Pose(1,0,0),5))
        self.assertFalse(gate.consider(Pose(1,0,0),7))  # missing keyframe
        self.assertFalse(gate.consider(Pose(4,0,0),8))  # inconsistent
        self.assertFalse(gate.consider(Pose(4,0,0),11)) # now too far in time
        gate.reset()
        self.assertIsNone(gate.pending)

if __name__=='__main__': unittest.main()
