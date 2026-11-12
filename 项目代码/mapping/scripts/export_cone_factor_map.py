#!/usr/bin/env python3
"""Save one latched cone-factor OccupancyGrid as ROS map_server-compatible PGM/YAML.

Example:
  python export_cone_factor_map.py --topic /cone_factor_graph/map \
      --output D:/ubuntu/slam/maps_replay/anchor_corrected

This listens to the independent anchor_map grid, NOT /map from SLAM Toolbox.
"""
import argparse
from pathlib import Path
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def write_pgm_yaml(message, output, *, live=False):
    prefix=Path(output).expanduser()
    prefix.parent.mkdir(parents=True,exist_ok=True)
    w,h=int(message.info.width),int(message.info.height)
    if w<=0 or h<=0 or len(message.data)!=w*h:
        raise ValueError('invalid OccupancyGrid')
    cells=np.asarray(message.data,dtype=np.int16).reshape((h,w))
    image=np.full((h,w),205,dtype=np.uint8)  # unknown gray
    image[cells==0]=254                      # free white
    image[cells>=65]=0                      # occupied black
    # ROS map origin lower left; PGM image origin upper left.
    image=image[::-1,:]
    pgm=prefix.with_suffix('.pgm')
    yaml=prefix.with_suffix('.yaml')
    image_path=pgm.with_suffix('.pgm.tmp') if live else pgm
    with image_path.open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(image.tobytes())
    if live:
        image_path.replace(pgm)
    x=float(message.info.origin.position.x)
    y=float(message.info.origin.position.y)
    q=message.info.origin.orientation
    import math
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    if live and yaml.exists():
        return pgm,yaml
    metadata_path=yaml.with_suffix('.yaml.tmp') if live else yaml
    metadata_path.write_text(
        f'image: {pgm.name}\nresolution: {message.info.resolution:.9g}\n'
        f'origin: [{x:.9g}, {y:.9g}, {yaw:.9g}]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n',
        encoding='utf-8')
    if live:
        metadata_path.replace(yaml)
    return pgm,yaml


class Collector(Node):
    def __init__(self, topic):
        super().__init__('cone_factor_grid_export')
        self.last=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,topic,self.received,qos)

    def received(self,msg):
        self.last=msg


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--topic',default='/cone_factor_graph/map')
    ap.add_argument('--output',required=True,help='PGM/YAML output prefix')
    ap.add_argument('--timeout',type=float,default=40.)
    args=ap.parse_args()
    rclpy.init()
    node=Collector(args.topic)
    try:
        end=time.monotonic()+max(1.,args.timeout)
        while node.last is None and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.2)
        if node.last is None:
            raise TimeoutError(f'No OccupancyGrid received on {args.topic}; check anchor_locked + grid_published')
        pgm,yaml=write_pgm_yaml(node.last,args.output)
        print(f'Saved {pgm} and {yaml} (frame={node.last.header.frame_id})',flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
