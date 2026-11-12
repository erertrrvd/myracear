import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from cone_factor_geom import Pose
from cone_factor_graph_core import ConeFactorGraph


class FieldBoundaryTests(unittest.TestCase):
    def test_inside_does_not_pull_trajectory_toward_wall(self):
        graph = ConeFactorGraph(bounds=(-2, 28, -5, 5), boundary_sigma_m=.1)
        graph.add_pose(Pose(2, 0, 0))
        before = graph.poses[-1]
        result = graph.optimize()
        self.assertTrue(result['ok'])
        self.assertAlmostEqual(graph.poses[-1].x, before.x, places=6)
        self.assertAlmostEqual(graph.poses[-1].y, before.y, places=6)

    def test_outside_is_softly_pulled_back_even_without_anchor_observations(self):
        graph = ConeFactorGraph(bounds=(-2, 28, -5, 5), boundary_sigma_m=.05)
        graph.add_pose(Pose(0, 6, 0))
        graph.optimize(iterations=15)
        self.assertLess(graph.poses[-1].y, 5.5)
        self.assertGreater(graph.poses[-1].y, 5.0)

    def test_invalid_bounds_rejected(self):
        with self.assertRaises(ValueError):
            ConeFactorGraph(bounds=(1, 31, -5, 5))
        with self.assertRaises(ValueError):
            ConeFactorGraph(bounds=(-2, 28, -5, 5), boundary_sigma_m=0)


if __name__ == '__main__':
    unittest.main()
