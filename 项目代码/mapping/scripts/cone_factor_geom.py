#!/usr/bin/env python3
"""Pure Python 2D cone geometry for an opt-in start-line landmark diagnostic.

No ROS dependencies. Points are in meters; angles in radians. This does not
alter SLAM Toolbox's internal scan matcher or publish map->odom.
"""
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt


def wrap(angle):
    return (angle + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def compose(a, b):
    c, s = cos(a.yaw), sin(a.yaw)
    return Pose(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
                wrap(a.yaw + b.yaw))


def inverse(p):
    c, s = cos(p.yaw), sin(p.yaw)
    return Pose(-c * p.x - s * p.y, s * p.x - c * p.y, wrap(-p.yaw))


def transform_point(p, pt):
    c, s = cos(p.yaw), sin(p.yaw)
    return (p.x + c * pt[0] - s * pt[1], p.y + s * pt[0] + c * pt[1])


def euclidean(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def scan_cones(ranges, amin, ainc, range_min, range_max, *,
               cluster_gap=0.20, min_points=3, min_width=0.035,
               max_width=0.38, max_cones=12):
    """Cluster *adjacent* rays; no repeated-frame point accumulation in motion.

    A cluster centroid is the visible surface centroid, not the true cone axis.
    Use a measured bias correction if centimeter-level accuracy is required.
    """
    groups, group = [], []
    for i, raw in enumerate(ranges):
        r = float(raw)
        if not isfinite(r) or r < range_min or r > range_max:
            if group:
                groups.append(group)
                group = []
            continue
        a = amin + i * ainc
        xy = (r * cos(a), r * sin(a))
        if group and euclidean(group[-1], xy) > cluster_gap:
            groups.append(group)
            group = []
        group.append(xy)
    if group:
        groups.append(group)
    # 360-degree scans: groups on both sides of the angle seam may be connected.
    if len(groups) >= 2 and len(ranges) > 5 and abs(abs(ainc) * (len(ranges) - 1) - 2*pi) < .1:
        if euclidean(groups[0][0], groups[-1][-1]) <= cluster_gap:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
    out = []
    for points in groups:
        if len(points) < min_points:
            continue
        # Maximum pairwise distance prevents a long wall from looking like a cone.
        w = max(euclidean(a, b) for a in points for b in points)
        if not min_width <= w <= max_width:
            continue
        x = sum(p[0] for p in points) / len(points)
        y = sum(p[1] for p in points) / len(points)
        out.append((x, y))
    return sorted(out, key=lambda p: hypot(*p))[:max_cones]


def triangle_area(a, b, c):
    return abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2.0


def noncollinear(points, min_area=.12):
    if len(points) < 3:
        return False
    return max(triangle_area(points[i], points[j], points[k])
               for i in range(len(points)) for j in range(i+1, len(points))
               for k in range(j+1, len(points))) >= min_area


class ReferenceBuilder:
    """Aggregate stationary detections in start frame, counting unique frames."""
    def __init__(self, merge_distance=0.24):
        self.merge_distance = merge_distance
        self.clusters = [] # [x, y, count]
        self.frames = 0

    def reset(self):
        self.clusters.clear()
        self.frames = 0

    def add(self, points):
        used = set()
        for x, y in points:
            candidates = [(hypot(x-c[0], y-c[1]), i) for i,c in enumerate(self.clusters)
                          if i not in used]
            best = min(candidates, default=(float('inf'), -1))
            if best[0] < self.merge_distance:
                c = self.clusters[best[1]]
                count = c[2] + 1
                c[0] += (x-c[0]) / count
                c[1] += (y-c[1]) / count
                c[2] = count
                used.add(best[1])
            else:
                self.clusters.append([x, y, 1])
                used.add(len(self.clusters)-1)
        self.frames += 1

    def finish(self, min_seen, max_refs=8):
        # Prefer repeatedly observed features; prune sporadic environmental returns.
        candidates = [c for c in self.clusters if c[2] >= min_seen]
        candidates.sort(key=lambda c: (-c[2], hypot(c[0], c[1])))
        return [(x, y) for x, y, _ in candidates[:max_refs]]


def fit_rigid(source, target):
    """Closed-form, proper 2D rigid transform source -> target (no reflection)."""
    n = len(source)
    if n != len(target) or n < 2:
        raise ValueError('Need >=2 matched point pairs')
    sx = sum(p[0] for p in source)/n
    sy = sum(p[1] for p in source)/n
    tx = sum(p[0] for p in target)/n
    ty = sum(p[1] for p in target)/n
    cross = dot = 0.0
    for a,b in zip(source, target):
        x,y=a[0]-sx,a[1]-sy
        X,Y=b[0]-tx,b[1]-ty
        cross += x*Y-y*X
        dot += x*X+y*Y
    theta=atan2(cross,dot)
    c,s=cos(theta),sin(theta)
    return Pose(tx-c*sx+s*sy,ty-s*sx-c*sy,theta)


def greedy_pairs(pose, observed, reference, gate):
    possibilities = []
    for i, point in enumerate(observed):
        transformed=transform_point(pose, point)
        for j, fixed in enumerate(reference):
            d=euclidean(transformed,fixed)
            if d <= gate:
                possibilities.append((d,i,j))
    possibilities.sort()
    matched_i, matched_j, pairs=set(),set(),[]
    for d,i,j in possibilities:
        if i not in matched_i and j not in matched_j:
            matched_i.add(i); matched_j.add(j)
            pairs.append((i,j))
    return pairs


def match_constellation(observed, reference, predicted, *, min_matches=3,
                        pair_gate=.22, inlier_gate=.23, max_rms=.13,
                        max_prior_m=1.25, max_prior_rad=.55,
                        ambiguity_margin=.018, min_area=.12):
    """Conservative pair-seeded, odometry-prior, RANSAC-like map alignment.

    Returns (pose, inliers, rms, status). Does NOT update a physical TF.
    Refuses ambiguous and non-collinear sets, especially repeated cone spacing.
    """
    if len(observed) < min_matches or len(reference) < min_matches:
        return None, 0, None, 'too_few_cones'
    hypotheses=[]
    for i in range(len(observed)):
        for j in range(i+1,len(observed)):
            p,q=observed[i],observed[j]
            dp=euclidean(p,q)
            if dp < .25:
                continue
            for u in range(len(reference)):
                for v in range(len(reference)):
                    if u==v:
                        continue
                    a,b=reference[u],reference[v]
                    if abs(dp-euclidean(a,b)) > pair_gate:
                        continue
                    th=wrap(atan2(b[1]-a[1],b[0]-a[0])-atan2(q[1]-p[1],q[0]-p[0]))
                    c,s=cos(th),sin(th)
                    candidate=Pose(a[0]-c*p[0]+s*p[1],a[1]-s*p[0]-c*p[1],th)
                    if hypot(candidate.x-predicted.x,candidate.y-predicted.y)>max_prior_m:
                        continue
                    if abs(wrap(candidate.yaw-predicted.yaw))>max_prior_rad:
                        continue
                    pairs=greedy_pairs(candidate, observed, reference, inlier_gate)
                    if len(pairs)<min_matches:
                        continue
                    src=[observed[ii] for ii,_ in pairs]
                    if not noncollinear(src,min_area):
                        continue
                    pose=fit_rigid(src,[reference[jj] for _,jj in pairs])
                    if hypot(pose.x-predicted.x,pose.y-predicted.y)>max_prior_m:
                        continue
                    if abs(wrap(pose.yaw-predicted.yaw))>max_prior_rad:
                        continue
                    pairs=greedy_pairs(pose, observed, reference, inlier_gate)
                    if len(pairs)<min_matches:
                        continue
                    src=[observed[ii] for ii,_ in pairs]
                    if not noncollinear(src,min_area):
                        continue
                    pose=fit_rigid(src,[reference[jj] for _,jj in pairs])
                    residual=sqrt(sum(euclidean(transform_point(pose,observed[ii]),reference[jj])**2
                                      for ii,jj in pairs)/len(pairs))
                    if residual>max_rms:
                        continue
                    # Slightly favor the odometric prior, never a large unexplained turn.
                    cost=residual + .010*hypot(pose.x-predicted.x,pose.y-predicted.y) + \
                         .015*abs(wrap(pose.yaw-predicted.yaw))
                    hypotheses.append((len(pairs),cost,residual,pose,tuple(sorted(pairs))))
    if not hypotheses:
        return None,0,None,'no_consistent_alignment'
    hypotheses.sort(key=lambda h:(-h[0],h[1]))
    best=hypotheses[0]
    for h in hypotheses[1:]:
        if h[0]<best[0]:
            break
        # Ignore equivalent pair seeds leading to the same actual transform.
        if hypot(h[3].x-best[3].x,h[3].y-best[3].y)<.10 and abs(wrap(h[3].yaw-best[3].yaw))<.07:
            continue
        if h[1]-best[1] < ambiguity_margin:
            return None,best[0],best[2],'ambiguous'
        break
    return best[3],best[0],best[2],'trusted'


class ConsecutiveCorrectionGate:
    """Require agreement of TWO different consecutive keyframes before global reset.

    The proposed correction maps original odom frame into the anchor frame,
    so its transform should be almost constant for successive observations.
    A one-off false match is insufficient to create a strong loop factor.
    """
    def __init__(self, distance_tol=.35, angle_tol=.16):
        self.distance_tol=float(distance_tol)
        self.angle_tol=float(angle_tol)
        self.pending=None

    def reset(self):
        self.pending=None

    def consider(self, correction, keyframe_id):
        prev=self.pending
        matches=(prev is not None and keyframe_id==prev[0]+1
                 and euclidean((correction.x,correction.y),
                               (prev[1].x,prev[1].y))<=self.distance_tol
                 and abs(wrap(correction.yaw-prev[1].yaw))<=self.angle_tol)
        if matches:
            self.reset()
            return True
        self.pending=(keyframe_id,correction)
        return False
