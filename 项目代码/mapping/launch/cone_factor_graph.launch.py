#!/usr/bin/env python3
"""Independent, read-only cone factor graph alongside existing SLAM Toolbox.

Before starting this, start the existing distributed mapping pipeline that
publishes /scan_pc and its scan-time odom_pc->base_pc/laser_pc TF.
Example (Windows ROS2 pixi shell):
  ros2 launch <path-to-this-file> log_path:=D:/ubuntu/slam/maps/anchor_test.jsonl
RViz Fixed Frame: anchor_map; add /cone_factor_graph/path and /odom_path.
"""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context):
    val=lambda n: LaunchConfiguration(n).perform(context)
    script=Path(__file__).resolve().parents[1] / 'scripts' / 'cone_factor_graph_node'
    args=['--ros-args',
          '-p',f'scan_topic:={val("scan_topic")}',
          '-p',f'odom_frame:={val("odom_frame")}',
          '-p',f'base_frame:={val("base_frame")}',
          '-p',f'log_path:={val("log_path")}',
          '-p',f'max_keyframes:={int(val("max_keyframes"))}',
          '-p',f'keyframe_distance:={float(val("keyframe_distance"))}',
          '-p',f'anchor_prior_gate_m:={float(val("anchor_prior_gate_m"))}',
          '-p',f'anchor_prior_gate_rad:={float(val("anchor_prior_gate_rad"))}',
          '-p',f'publish_occupancy:={val("publish_occupancy")}',
          '-p',f'grid_resolution:={float(val("grid_resolution"))}',
          '-p',f'grid_half_size:={float(val("grid_half_size"))}',
          '-p',f'map_publish_period:={float(val("map_publish_period"))}',
          '-p',f'allow_global_relocalization:={val("allow_global_relocalization")}',
          '-p',f'ref_frames:={int(val("ref_frames"))}',
          '-p',f'min_refs:={int(val("min_refs"))}']
    return [ExecuteProcess(cmd=[sys.executable,str(script),*args],output='screen')]


def generate_launch_description():
    params={'scan_topic':'/scan_pc','odom_frame':'odom_pc','base_frame':'base_pc',
            'log_path':'','max_keyframes':'500','keyframe_distance':'0.65',
            'anchor_prior_gate_m':'0.8','anchor_prior_gate_rad':'0.35',
            'publish_occupancy':'true','grid_resolution':'0.10',
            'grid_half_size':'35.0','map_publish_period':'12.0',
            'allow_global_relocalization':'false','ref_frames':'20','min_refs':'4'}
    return LaunchDescription([
        *(DeclareLaunchArgument(k,default_value=v) for k,v in params.items()),
        OpaqueFunction(function=_setup),
    ])
