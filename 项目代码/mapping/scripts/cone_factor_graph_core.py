#!/usr/bin/env python3
"""2D pose/point-landmark factor graph; ROS-independent, sparse Gauss-Newton.

Frame: anchor_map = robot base pose when the start reference was locked.
Factors: relative wheel/IMU odometry, fixed start-cone observations, reobserved
non-anchor cone observations. No GTSAM dependency or TF publisher.
This is an experimental factor-graph implementation, NOT the paper's conic
section fitting, and NOT a substitute for in-vehicle safety validation.
"""
from dataclasses import dataclass
from math import cos, sin, hypot, sqrt
import numpy as np
from scipy.sparse import coo_matrix, eye
from scipy.sparse.linalg import spsolve
from cone_factor_geom import Pose, compose, inverse, wrap


def between(a, b):
    return compose(inverse(a), b)


@dataclass
class Landmark:
    x: float
    y: float
    fixed: bool = False
    seen: int = 0
    last_pose: int = -1


@dataclass
class Observation:
    pose_id: int
    landmark_id: int
    local: tuple  # cone center measured in BASE frame, not laser frame
    sigma: float


class ConeFactorGraph:
    """Pose 0 and the start-cone landmarks are fixed to eliminate gauge freedom.

    Only call optimize on a snapshot from a worker/timer, not from a 10 Hz
    callback on the racing controller. Observations are assumed pre-associated
    and must be guarded with geometric and odometry consistency checks.
    """
    def __init__(self, odom_sigma_m=.075, odom_sigma_rad=.035,
                 landmark_sigma_m=.14, anchor_sigma_m=.085, bounds=None,
                 boundary_sigma_m=.25):
        self.poses = [Pose()]
        self.odometry = []
        self.landmarks = []
        self.observations = []
        self.odom_sigma_m = float(odom_sigma_m)
        self.odom_sigma_rad = float(odom_sigma_rad)
        self.landmark_sigma_m = float(landmark_sigma_m)
        self.anchor_sigma_m = float(anchor_sigma_m)
        if bounds is not None:
            if len(bounds) != 4 or not all(np.isfinite(bounds)):
                raise ValueError('bounds must be finite xmin, xmax, ymin, ymax')
            xmin, xmax, ymin, ymax = map(float, bounds)
            if xmin >= xmax or ymin >= ymax or not (xmin <= 0 <= xmax and ymin <= 0 <= ymax):
                raise ValueError('bounds must contain the start pose and have positive size')
            bounds = (xmin, xmax, ymin, ymax)
        if not np.isfinite(boundary_sigma_m) or boundary_sigma_m <= 0:
            raise ValueError('boundary_sigma_m must be positive')
        self.bounds = bounds
        self.boundary_sigma_m = float(boundary_sigma_m)
        self.last_solution = None

    def add_pose(self, measured_relative, seed=None):
        if not isinstance(measured_relative, Pose):
            raise TypeError('measured_relative must be Pose')
        self.poses.append(seed if seed is not None else compose(self.poses[-1], measured_relative))
        self.odometry.append(measured_relative)
        return len(self.poses) - 1

    def add_landmark(self, point, fixed=False):
        x, y = point
        if not np.isfinite(x) or not np.isfinite(y):
            raise ValueError('invalid landmark')
        self.landmarks.append(Landmark(float(x), float(y), bool(fixed)))
        return len(self.landmarks) - 1

    def add_observation(self, pose_id, landmark_id, observed_base, sigma=None):
        if not (0 <= pose_id < len(self.poses) and 0 <= landmark_id < len(self.landmarks)):
            raise IndexError('invalid pose or landmark')
        sigma = float(sigma if sigma is not None else (self.anchor_sigma_m if self.landmarks[landmark_id].fixed else self.landmark_sigma_m))
        if sigma <= 0:
            raise ValueError('sigma must be positive')
        self.observations.append(Observation(pose_id, landmark_id, tuple(map(float, observed_base)), sigma))
        lm = self.landmarks[landmark_id]
        lm.seen += 1
        lm.last_pose = pose_id

    def _layout(self):
        off = 3 * (len(self.poses) - 1)
        landmark_cols = {}
        for j, lm in enumerate(self.landmarks):
            if not lm.fixed:
                landmark_cols[j] = off
                off += 2
        return off, landmark_cols

    def _pack(self, lmcols):
        n = 3*(len(self.poses)-1)+2*len(lmcols)
        x=np.zeros(n)
        for i, p in enumerate(self.poses[1:]):
            x[3*i:3*i+3]=(p.x, p.y, p.yaw)
        for j, k in lmcols.items():
            lm=self.landmarks[j]
            x[k:k+2]=(lm.x, lm.y)
        return x

    def _state(self, x, lmcols):
        pp=[self.poses[0]]
        for i in range(1,len(self.poses)):
            k=3*(i-1)
            pp.append(Pose(float(x[k]),float(x[k+1]),wrap(float(x[k+2]))))
        ll=[(lm.x,lm.y) if lm.fixed else (float(x[lmcols[j]]),float(x[lmcols[j]+1]))
            for j,lm in enumerate(self.landmarks)]
        return pp,ll

    def _residual_jac(self,x,lmcols,derivatives=True):
        poses,lms=self._state(x,lmcols)
        rs=[]; ij=[]; jj=[]; vv=[]
        def add(r, derivatives_by_block):
            row=len(rs)
            rs.extend(r)
            if derivatives:
                for col, mat in derivatives_by_block:
                    if col is None: continue
                    for row_offset in range(len(r)):
                        for column_offset in range(len(mat[row_offset])):
                            val=mat[row_offset][column_offset]
                            if val:
                                ij.append(row+row_offset);jj.append(col+column_offset);vv.append(val)
        st=self.odom_sigma_m; sr=self.odom_sigma_rad
        for i,z in enumerate(self.odometry):
            a,b=poses[i],poses[i+1]
            c,s=cos(a.yaw),sin(a.yaw)
            dx,dy=b.x-a.x,b.y-a.y
            px,py=c*dx+s*dy,-s*dx+c*dy
            residual=((px-z.x)/st,(py-z.y)/st,wrap(b.yaw-a.yaw-z.yaw)/sr)
            ja=[[-c/st,-s/st, py/st],[s/st,-c/st,-px/st],[0,0,-1/sr]]
            jb=[[c/st,s/st,0],[-s/st,c/st,0],[0,0,1/sr]]
            add(residual,[(3*(i-1) if i>0 else None,ja),(3*i,jb)])
        for ob in self.observations:
            p=poses[ob.pose_id]; qx,qy=ob.local; lx,ly=lms[ob.landmark_id]
            c,s=cos(p.yaw),sin(p.yaw)
            ux,uy=c*qx-s*qy,s*qx+c*qy
            sig=ob.sigma
            rx=(p.x+ux-lx)/sig;ry=(p.y+uy-ly)/sig
            # 锚点也使用鲁棒核，减轻误关联造成的整图拉扯。
            norm=hypot(rx,ry)
            weight=1.0 if norm<=2.5 else sqrt(2.5/norm)
            pa=[[weight/sig,0.,weight*(-uy)/sig],[0.,weight/sig,weight*ux/sig]]
            la=[[-weight/sig,0.],[0.,-weight/sig]]
            add((weight*rx,weight*ry),[(3*(ob.pose_id-1) if ob.pose_id>0 else None,pa),
                                        (lmcols.get(ob.landmark_id),la)])
        if self.bounds is not None:
            xmin, xmax, ymin, ymax = self.bounds
            inv = 1.0 / self.boundary_sigma_m
            for i, p in enumerate(poses[1:], start=1):
                # 单侧软约束：只惩罚车体参考点越过场地边界，不吸附到墙面。
                for value, lower, upper, col in ((p.x, xmin, xmax, 0), (p.y, ymin, ymax, 1)):
                    if value < lower:
                        residual = (value - lower) * inv
                    elif value > upper:
                        residual = (value - upper) * inv
                    else:
                        residual = 0.0
                    jac = [[0.0, 0.0, 0.0]]
                    if residual:
                        jac[0][col] = inv
                    add((residual,), [(3*(i-1), jac)])
        result=np.asarray(rs,dtype=float)
        if derivatives:
            return result,coo_matrix((vv,(ij,jj)),shape=(len(result),len(x))).tocsr()
        return result,None

    def optimize(self, iterations=8, damping=.005):
        if len(self.poses)<=1 or (len(self.observations)==0 and self.bounds is None):
            return {'ok':False,'reason':'not_enough_constraints'}
        n,lmcols=self._layout()
        x=self._pack(lmcols)
        initial=np.linalg.norm(self._residual_jac(x,lmcols,False)[0])
        iters=0
        for _ in range(iterations):
            r,j=self._residual_jac(x,lmcols)
            cost=float(np.dot(r,r))
            lhs=(j.T @ j + damping*eye(n,format='csr')).tocsc()
            rhs=-j.T@r
            try: dx=spsolve(lhs,rhs)
            except Exception: break
            if not np.all(np.isfinite(dx)): break
            accepted=False
            for scale in (1.,.5,.25,.125,.0625):
                candidate=x+dx*scale
                rr,_=self._residual_jac(candidate,lmcols,False)
                if float(np.dot(rr,rr)) < cost:
                    x=candidate;accepted=True;iters+=1;break
            if not accepted or np.linalg.norm(dx)<1e-6:break
        poses,lms=self._state(x,lmcols)
        self.poses=poses
        for i,point in enumerate(lms):
            if not self.landmarks[i].fixed:
                self.landmarks[i].x,self.landmarks[i].y=point
        after=float(np.linalg.norm(self._residual_jac(x,lmcols,False)[0]))
        self.last_solution={'ok':True,'n_poses':len(poses),'n_landmarks':len(lms),
                            'n_factors':len(self.odometry)+len(self.observations),
                            'weighted_error_before':round(float(initial),2),
                            'weighted_error_after':round(after,2),'iterations':iters}
        return self.last_solution

    def anchor_error(self):
        errors=[]
        for ob in self.observations:
            lm=self.landmarks[ob.landmark_id]
            if not lm.fixed:continue
            p=self.poses[ob.pose_id];qx,qy=ob.local
            c,s=cos(p.yaw),sin(p.yaw)
            errors.append(hypot(p.x+c*qx-s*qy-lm.x,p.y+s*qx+c*qy-lm.y))
        return None if not errors else sqrt(sum(e*e for e in errors)/len(errors))
