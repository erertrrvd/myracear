#!/bin/bash
# 用法：solid_test.sh <车速> <时长秒>   在 D:\ubuntu\slam 的 pixi 环境里跑一次模拟建图 + 保存
SPEED=${1:-1.5}; DUR=${2:-26}
cd /d/ubuntu/slam || exit 1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 PYTHONIOENCODING=utf-8
T=$(date +%H%M%S)
DIR=D:/ubuntu/slam/maps_test_solid_$T
PROJ="C:/Users/lzc2/Desktop/my_racecar（复件）"
SIM="D:/VSCode/ros2技术集/智能车总文件/my_racecar（复件）/事项/临时修改/windows建图linux建图分布式通信/windows/模拟测试/fake_car3_锥桶和走动物体(车速可调).py"
pixi run --frozen ros2 launch "$PROJ/mapping/launch/distributed_mapping_pc_windows.launch.py" map_dir:=$DIR rviz:=false autosave_period:=0 > /tmp/solid_launch.log 2>&1 &
sleep 12
pixi run --frozen python "$SIM" $SPEED $DUR > /tmp/solid_car.log 2>&1
sleep 3
pixi run --frozen python "$PROJ/mapping/scripts/save_mapping" --run-index 1 --map-dir $DIR 2>&1 | tail -3
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { \$_.Name -match '^(python|ros2|async_slam_toolbox_node|pixi)' -and \$_.CommandLine -match 'maps_test_solid|async_slam_toolbox|scan_tf_bridge' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force -ErrorAction SilentlyContinue }" > /dev/null 2>&1
echo "DIR=$DIR"
ls $DIR/run1
grep -n -i "obstacle_layer\|Traceback\|Error" /tmp/solid_launch.log | head -10
