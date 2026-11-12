import sys
import unittest
from pathlib import Path
SCRIPTS=Path(__file__).resolve().parents[2]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from cone_factor_geom import Pose
from cone_factor_map import ScanFrame, OccupancyReprojector, bresenham


class OccupancyTest(unittest.TestCase):
    def test_unconfirmed_hit_stays_unknown_until_reobserved(self):
        mapper=OccupancyReprojector(resolution=.2,half_size=7,min_hits=2,
                                    min_hit_ratio=.35,unconfirmed_unknown=True)
        x,y=mapper.cell((2,0))
        frame=ScanFrame((0,0),[(2,0)])
        self.assertEqual(mapper.render([Pose()],[frame]).cells[y,x],-1)
        self.assertEqual(mapper.render([Pose(),Pose()],[frame,frame]).cells[y,x],100)
        clear=ScanFrame((0,0),[(4,0)])
        self.assertEqual(mapper.render([Pose(),Pose()],[frame,clear]).cells[y,x],0)

    def test_stricter_ratio_removes_weak_hits(self):
        frames=[ScanFrame((0,0),[(2,0)]) for _ in range(3)] + [ScanFrame((0,0),[(4,0)]) for _ in range(7)]
        poses=[Pose() for _ in frames]
        loose=OccupancyReprojector(resolution=.2,half_size=7,min_hit_ratio=.22)
        strict=OccupancyReprojector(resolution=.2,half_size=7,min_hit_ratio=.35)
        x,y=loose.cell((2,0))
        self.assertEqual(int(loose.render(poses,frames).cells[y,x]),100)
        self.assertEqual(int(strict.render(poses,frames).cells[y,x]),0)

    def test_grid_ray(self):
        self.assertEqual(list(bresenham(0,0,3,0)),[(0,0),(1,0),(2,0),(3,0)])

    def test_v2_requires_hits_and_ratio(self):
        m=OccupancyReprojector(resolution=.2,half_size=7,min_hits=2,min_hit_ratio=.5)
        frames=[ScanFrame((0,0),[(2,0)])] + [ScanFrame((0,0),[(4,0)]) for _ in range(5)]
        cells=m.render([Pose() for _ in frames],frames).cells
        x,y=m.cell((2,0))
        self.assertEqual(int(cells[y,x]),0)
        frames.insert(0,ScanFrame((0,0),[(2,0)]))
        cells=m.render([Pose() for _ in frames],frames).cells
        self.assertEqual(int(cells[y,x]),0)
        frames=[ScanFrame((0,0),[(2,0)]) for _ in range(2)]
        cells=m.render([Pose() for _ in frames],frames).cells
        self.assertEqual(int(cells[y,x]),100)

    def test_optimized_history_reprojects_obstacle_without_ghosts(self):
        mapper=OccupancyReprojector(resolution=.2,half_size=8,min_hits=1)
        # Both scans measured the SAME obstacle from slightly different robot poses.
        raw_poses=[Pose(),Pose(1.0,0,0)]
        corrected=[Pose(),Pose(0.0,0,0)]
        obs=[ScanFrame((0,0),[(2,0)]),ScanFrame((0,0),[(2,0)])]
        before=mapper.render(raw_poses,obs)
        after=mapper.render(corrected,obs)
        old=mapper.cell((3.0,0.0));expected=mapper.cell((2.0,0.0))
        self.assertEqual(int(before.cells[old[1],old[0]]),100)
        self.assertEqual(int(after.cells[old[1],old[0]]),-1) # no stale occupancy
        self.assertEqual(int(after.cells[expected[1],expected[0]]),100)
        self.assertEqual(after.n_scans,2)

    def test_ray_freespace_and_unknown(self):
        m=OccupancyReprojector(resolution=.2,half_size=7,min_hits=1)
        grid=m.render([Pose()],[ScanFrame((0,0),[(2,0)])]).cells
        near=m.cell((.6,0));endpoint=m.cell((2.,0));outside=m.cell((-4.,-4.))
        self.assertEqual(int(grid[near[1],near[0]]),0)
        self.assertEqual(int(grid[endpoint[1],endpoint[0]]),100)
        self.assertEqual(int(grid[outside[1],outside[0]]),-1)

    def test_live_ratio_updates(self):
        mapper=OccupancyReprojector(min_hits=1,min_hit_ratio=.35)
        cache={}
        poses=[]
        frames=[]
        x,y=mapper.cell((2,0))
        for distance,expected in [(2,100),(3,100),(3,0),(2,100),(3,100),(3,0)]:
            poses.append(Pose())
            frames.append(ScanFrame((0,0),[(distance,0)]))
            result=mapper.render(poses,frames,cache)
            self.assertEqual(result.cells[y,x],expected)
        poses[0]=Pose(1,0,0)
        self.assertTrue((mapper.render(poses,frames,cache).cells==mapper.render(poses,frames).cells).all())

    def test_lidar_extrinsic_origin(self):
        m=OccupancyReprojector(resolution=.2,half_size=7,min_hits=1)
        grid=m.render([Pose(1.0,0,0)], [ScanFrame((.4,0),[(2.4,0)])]).cells
        last=m.cell((3.4,0.0))
        self.assertEqual(int(grid[last[1],last[0]]),100)

    def test_invalid_frames_rejected(self):
        with self.assertRaises(ValueError):
            OccupancyReprojector().render([Pose()],[])
        with self.assertRaises(ValueError):
            OccupancyReprojector(resolution=.02)

if __name__=='__main__': unittest.main()
