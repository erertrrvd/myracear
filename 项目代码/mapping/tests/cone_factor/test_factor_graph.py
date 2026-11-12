import math
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2] / 'scripts'
sys.path.insert(0,str(ROOT))
from cone_factor_geom import Pose, transform_point, inverse, match_constellation, wrap
from cone_factor_graph_core import ConeFactorGraph,between

class FactorGraphTest(unittest.TestCase):
    def test_pose_inverse_and_between(self):
        a=Pose(2,3,0.7);b=Pose(3,4,1.2)
        r=between(a,b)
        from cone_factor_geom import compose
        actual=compose(a,r)
        self.assertAlmostEqual(actual.x,b.x,places=8)
        self.assertAlmostEqual(actual.y,b.y,places=8)
        self.assertAlmostEqual(wrap(actual.yaw-b.yaw),0,places=8)

    def test_3mps_full_loop_anchored_optimization(self):
        # 20 seconds at 3 m/s on a circular track, 10 Hz pose observations.
        # No noise on cones, deliberate systematic heading bias in odometry.
        n=200; dt=.1;v=3.0; radius=v*n*dt/(2*math.pi)
        truth=[Pose(radius*math.sin(2*math.pi*k/n),
                    radius*(1-math.cos(2*math.pi*k/n)),
                    wrap(2*math.pi*k/n)) for k in range(n+1)]
        anchors=[(-1.4,-.6),(-1.2,1.3),(1.6,-1.1),(1.2,1.9)]
        g=ConeFactorGraph(odom_sigma_m=.06,odom_sigma_rad=.025,anchor_sigma_m=.07)
        ids=[g.add_landmark(z,fixed=True) for z in anchors]
        def observe(k):
            for j,lm in enumerate(anchors):
                z=transform_point(inverse(truth[k]),lm)
                g.add_observation(k,ids[j],z)
        observe(0)
        for k in range(1,n+1):
            od=between(truth[k-1],truth[k])
            measured=Pose(od.x*1.025,od.y*1.025,wrap(od.yaw+.0018))
            g.add_pose(measured)
            if k in (1,2,n-4,n-3,n-2,n-1,n): observe(k)
        prior_end=g.poses[-1];prior_err=math.hypot(prior_end.x-truth[-1].x,prior_end.y-truth[-1].y)
        prior_heading=abs(wrap(prior_end.yaw-truth[-1].yaw))
        self.assertGreater(prior_err,1.0)
        result=g.optimize(iterations=12)
        post_end=g.poses[-1];post_err=math.hypot(post_end.x-truth[-1].x,post_end.y-truth[-1].y)
        post_heading=abs(wrap(post_end.yaw-truth[-1].yaw))
        print('3m/s closed lap: prior %.3fm %.1fdeg; after %.3fm %.1fdeg; mid-change %.3fm' % (
           prior_err,math.degrees(prior_heading),post_err,math.degrees(post_heading),
           math.hypot(g.poses[n//2].x-truth[n//2].x,g.poses[n//2].y-truth[n//2].y)))
        self.assertTrue(result['ok'])
        self.assertLess(post_err,.24)
        self.assertLess(post_heading,.05)
        self.assertLess(post_err,prior_err*.20)
        self.assertLess(g.anchor_error(),.15)

    def test_noisy_anchor_relocalization_with_outliers(self):
        import random
        rng=random.Random(7)
        anchors=[(-1.4,-.6),(-1.2,1.3),(1.6,-1.1),(1.2,1.9)]
        actual=Pose(.13,-.09,.07)
        observed=[transform_point(inverse(actual),p) for p in anchors]
        observed=[(x+rng.gauss(0,.018),y+rng.gauss(0,.018)) for x,y in observed]
        observed += [(5.0,4.5),(-3.0,4.0)] # extra non-cone detections
        predicted=Pose(1.3,-.2,.3)
        fit,n,rms,status=match_constellation(observed,anchors,predicted,
                    max_prior_m=3.5,max_prior_rad=.95)
        self.assertEqual(status,'trusted')
        self.assertEqual(n,4)
        self.assertLess(math.hypot(fit.x-actual.x,fit.y-actual.y),.12)
        self.assertLess(abs(wrap(fit.yaw-actual.yaw)),.12)
        self.assertLess(rms,.10)

    def test_anchor_not_observed_cannot_resolve_drift(self):
        g=ConeFactorGraph()
        a=g.add_landmark((1.,1.),fixed=True)
        g.add_observation(0,a,(1.,1.))
        for _ in range(30):g.add_pose(Pose(.33,0,.01))
        x=g.poses[-1].x;y=g.poses[-1].y
        g.optimize()
        self.assertAlmostEqual(g.poses[-1].x,x,places=4)
        self.assertAlmostEqual(g.poses[-1].y,y,places=4)

    def test_ambiguous_square_constellation_rejected_without_prior(self):
        reference=[(-1,-1),(-1,1),(1,1),(1,-1)]
        result=match_constellation(reference,reference,Pose(),max_prior_rad=math.pi,max_prior_m=2.5,ambiguity_margin=.05)
        self.assertNotEqual(result[3],'trusted')

if __name__=='__main__':unittest.main()
