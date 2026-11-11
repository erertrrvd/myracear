#!/bin/bash
# 只读检查：不改任何东西、不装东西、不杀进程。
# 用法（小车上，root）：  bash check_env.sh
# 结果同时写到 /tmp/car_check.txt，把输出整段贴给我。
{
echo "===== 1 系统/硬件"; uname -a; head -3 /etc/os-release; free -h; df -h / /home | cat; echo "CPU 核数: $(nproc)"
echo "===== 2 昇腾板卡"; npu-smi info 2>&1 | head -25
echo "===== 3 CANN"; ls /usr/local/Ascend 2>&1; ls /usr/local/Ascend/ascend-toolkit 2>&1
cat /usr/local/Ascend/ascend-toolkit/latest/ascend_toolkit_install.info 2>&1 | head -5
ls -l /usr/local/Ascend/ascend-toolkit/set_env.sh 2>&1
echo "===== 4 atc 能否找到"; bash -c 'source /usr/local/Ascend/ascend-toolkit/set_env.sh 2>&1; which atc; atc --version 2>&1 | head -3'
echo "===== 5 root 下 python3（推理用这个）"; which python3; python3 --version
for m in torch torchvision cv2 numpy yaml tqdm pycocotools ais_bench onnx onnxruntime acl; do
  python3 -c "import $m; print('$m OK', getattr($m,'__version__',''))" 2>&1 | tail -1
done
echo "===== 6 /usr/bin/python3（ROS 用，第二阶段）"; /usr/bin/python3 --version
for m in rclpy cv2 numpy; do /usr/bin/python3 -c "import $m; print('$m OK')" 2>&1 | tail -1; done
echo "===== 7 摄像头"; ls -l /dev/video* 2>&1; v4l2-ctl --list-devices 2>&1 | head -10
echo "谁在占用 /dev/video0:"; fuser -v /dev/video0 2>&1 || echo "(没有进程占用，或没有 fuser)"
echo "===== 8 正在运行的相关进程"; ps aux | grep -E "ros2|python|camera|usb_cam|rviz|light" | grep -v grep | cut -c1-150
echo "===== 9 目录"; ls /home/racecar/ /home/racecar/src/ 2>&1; ls /home/racecar/src/my_racecar/ 2>&1
echo "===== 10 已有的 .om / 课程工程"; find / -maxdepth 4 \( -name "infer_project*" -o -name "*.om" -o -name "onnx2om.py" \) 2>/dev/null | head -20
echo "===== 11 ROS"; source /opt/ros/humble/setup.bash 2>&1; echo "ROS_DISTRO=$ROS_DISTRO"; ros2 topic list 2>&1 | head -20
echo "===== 12 联网/pip"; ping -c 1 -W 2 pypi.org 2>&1 | tail -2; pip --version 2>&1; pip config list 2>&1 | head -3
echo "===== 13 X11（弹窗用）"; echo "DISPLAY=$DISPLAY"
} 2>&1 | tee /tmp/car_check.txt
