#!/usr/bin/env python3
"""Read-only 2D occupancy reconstruction from historical scan keyframes.

ROS-independent. Every rendering starts from empty vote arrays and uses CURRENT
optimized poses; old scan pixels cannot remain at their previous wrong poses.
No map->odom TF is published. Rays from valid laser returns provide free cells;
unknown/no-return beams are not inferred to be free.
"""
from dataclasses import dataclass
from math import floor, hypot
import numpy as np
from cone_factor_geom import transform_point


@dataclass
class ScanFrame:
    """One keyframe's scan in the robot base frame."""
    origin: tuple  # lidar origin in base frame
    hits: list     # lidar endpoints in base frame


@dataclass
class OccupancyResult:
    cells: np.ndarray  # [height, width], int8, -1 unknown, 0 free, 100 occupied
    resolution: float
    origin_x: float
    origin_y: float
    n_scans: int
    n_hits: int

    @property
    def width(self):
        return self.cells.shape[1]

    @property
    def height(self):
        return self.cells.shape[0]


def bresenham(x0, y0, x1, y1):
    """Grid cell ray including start and endpoint (integer coordinates)."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class OccupancyReprojector:
    def __init__(self, resolution=0.10, half_size=35.0, min_hits=2,
                 min_hit_ratio=0.22, max_ray_length=12.0, unconfirmed_unknown=False):
        self.res = float(resolution)
        self.half = float(half_size)
        self.min_hits = int(min_hits)
        self.min_hit_ratio = float(min_hit_ratio)
        self.max_ray_length = float(max_ray_length)
        self.unconfirmed_unknown = bool(unconfirmed_unknown)
        if not (0.04 <= self.res <= .5 and 5.0 <= self.half <= 90.0):
            raise ValueError('unreasonable map geometry')
        self.size = int(round(2 * self.half / self.res))
        if self.size > 1800:
            raise ValueError('occupancy map size too large; raise resolution or reduce half_size')

    def cell(self, xy):
        col = int(floor((xy[0] + self.half) / self.res))
        row = int(floor((xy[1] + self.half) / self.res))
        return (col, row) if 0 <= col < self.size and 0 <= row < self.size else None

    def render(self, poses, frames, cache=None):
        """Rebuild a map with the graph's latest optimized *historical* poses.

        A valid ray with an endpoint outside the extent is skipped rather than
        erroneously marking the clipped boundary as an obstacle.
        Multiple rays from the same keyframe cast one vote per grid cell.
        """
        if len(poses) != len(frames):
            raise ValueError('one scan frame per keyframe pose is required')
        side = self.size
        # 位姿未变时只累加新扫描；历史位姿被优化后从头重投。
        if cache and poses[:len(cache['poses'])] == cache['poses']:
            hit, free, samples = cache['hit'], cache['free'], cache['samples']
            start = len(cache['poses'])
        else:
            hit = np.zeros((side, side), dtype=np.int32)
            free = np.zeros_like(hit)
            samples = start = 0
        for pose, frame in zip(poses[start:], frames[start:]):
            if frame is None or not frame.hits:
                continue
            source = transform_point(pose, frame.origin)
            src = self.cell(source)
            if src is None:
                continue
            this_hit = set()
            this_free = set()
            for q in frame.hits:
                if hypot(q[0] - frame.origin[0], q[1] - frame.origin[1]) > self.max_ray_length:
                    continue
                end = self.cell(transform_point(pose, q))
                if end is None:
                    continue
                samples += 1
                ray = bresenham(*src, *end)
                last = next(ray)
                for current in ray:
                    this_free.add(last[1] * side + last[0])
                    last = current
                this_hit.add(end[1] * side + end[0])
            this_free.difference_update(this_hit)
            if this_free:
                idx = np.fromiter(this_free, dtype=np.int64)
                free.reshape(-1)[idx] += 1
            if this_hit:
                idx = np.fromiter(this_hit, dtype=np.int64)
                hit.reshape(-1)[idx] += 1
        # V2 原始投票：需要足够命中次数，并满足命中占全部观测的比例。
        observed = (hit + free) > 0
        cells = np.full((side,side),-1,dtype=np.int8)
        cells[observed] = 0
        occupied = (hit >= self.min_hits) & (hit >= self.min_hit_ratio * np.maximum(1,hit+free))
        cells[occupied] = 100
        # 刚启动只有少量关键帧时先显示已扫过的空地，后续帧够多再使用投票判定。
        if self.unconfirmed_unknown:
            cells[(hit > 0) & (hit < self.min_hits) & (free == 0)] = -1
        elif self.min_hits > 1 and len(frames) < 3:
            cells[(hit == 0) & (free > 0)] = 0
            cells[hit > 0] = 100
        if cache is not None:
            cache.update(poses=list(poses), hit=hit, free=free, samples=samples)
        return OccupancyResult(cells, self.res, -self.half, -self.half,
                               len(frames), samples)
