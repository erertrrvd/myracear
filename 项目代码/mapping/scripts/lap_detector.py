import math


class LapDetector:
    """按里程、离开起点、回到起点和同向四项条件判定一圈。"""
    def __init__(self, min_distance=20.0, depart_distance=5.0, return_radius=1.5,
                 heading_tolerance=0.6, min_seconds=15.0):
        self.min_distance=min_distance; self.depart_distance=depart_distance
        self.return_radius=return_radius; self.heading_tolerance=heading_tolerance
        self.min_seconds=min_seconds; self.start=None; self.last=None
        self.distance=0.0; self.departed=False; self.done=False

    def update(self,t,x,y,yaw):
        if not all(math.isfinite(v) for v in (t,x,y,yaw)) or self.done: return False
        if self.start is None:
            self.start=(t,x,y,yaw); self.last=(t,x,y); return False
        dt=t-self.last[0]
        if dt <= 0: return False
        step=math.hypot(x-self.last[1],y-self.last[2]); self.last=(t,x,y)
        if step>max(1.0,4.0*dt): return False
        self.distance+=step
        dist=math.hypot(x-self.start[1],y-self.start[2])
        if dist>=self.depart_distance: self.departed=True
        angle=abs(math.atan2(math.sin(yaw-self.start[3]),math.cos(yaw-self.start[3])))
        if self.departed and self.distance>=self.min_distance and t-self.start[0]>=self.min_seconds and dist<=self.return_radius and angle<=self.heading_tolerance:
            self.done=True; return True
        return False
