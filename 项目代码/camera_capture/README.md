# camera_capture：摄像头采集 + 红绿灯识别 + ROS2 话题

小车上的位置：`/home/racecar/src/my_racecar/camera_capture/`

## 文件

| 文件 | 作用 | 用哪个 python |
|---|---|---|
| `record_video.py` | 录视频（采集训练数据） | `/usr/bin/python3` |
| `check_env.sh` | 只读检查小车环境 | — |
| `convert.sh` | 把 `yolov5n.onnx` 转成 `output/yolov5n_bs1.om`（约 2 小时） | root 的 `python3`（conda） |
| `light_infer.py` | 识别：摄像头 → YOLOv5n → 状态，每帧写 `/tmp/light_state.txt` | root 的 `python3`（conda，有 ais_bench） |
| `light_bridge.py` | 读状态文件，发布 ROS2 话题 | `/usr/bin/python3`（ROS） |
| `light_listener.py` | 订阅话题并打印（接收端示例） | `/usr/bin/python3`（ROS） |
| `start_vision.sh` | 一键启动识别 + 话题发布 | — |
| `run.sh` | 只跑识别，不发话题（调试用） | — |
| `det_utils.py`、`onnx2om.py`、`config.yaml`、`common/` | 课程自带的后处理和转换脚本 | — |

识别和 ROS 分成两个进程，是因为 ais_bench（conda Python 3.9）和 rclpy（Python 3.10）不能在同一个进程里。

## 话题（验证阶段：只发布识别结果，不影响小车运动）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/traffic_state` | `std_msgs/String` | `red` / `green` / `off` / `none` / `conflict` / `stale` |
| `/traffic_light_box` | `std_msgs/Float32MultiArray` | `[x1, y1, x2, y2, 灯箱置信度, 亮灯置信度, 图宽, 图高, 结果延迟秒, 推理fps]`，没灯箱时框为 -1 |

- 默认 20 Hz 发布。
- `stale`：状态文件超过 0.5 秒没更新（推理停了或卡了）。
- 这两个话题现在**没有任何运动节点订阅**，所以开着识别也不会影响小车行驶。
- 日志里的 `GO` / `STOP` 只是显示，没有发出去。

**暂时关闭的 `/traffic_light`**（`std_msgs/Int32`，1 = 绿灯可以走）：`pure_track_test` 订阅了它（收到 1 会提前结束红灯等待），老系统的 `get_flag.cpp` 也订阅它（当成红灯面积）。验证阶段不发，避免干扰运动。接入控制时，把 `light_bridge.py` 和 `light_listener.py` 里标了 `[控制]` 的几行取消注释即可。

## 第一次部署

电脑 PowerShell（输入车的 SSH 密码）：

```bash
scp -r "C:\Users\lzc2\Desktop\my_racecar（复件）\camera_capture" root@192.168.5.100:/home/racecar/src/my_racecar/
scp "D:\VSCode\ros2技术集\智能车总文件\视觉模型\模型\yolov5n_best.onnx" root@192.168.5.100:/home/racecar/src/my_racecar/camera_capture/yolov5n.onnx
```

小车（root）：

```bash
cd /home/racecar/src/my_racecar/camera_capture
bash check_env.sh        # 只读检查，输出贴给我
bash convert.sh          # 确认环境没问题再做；后台转换，tail -f convert.log 看进度
ls -lh output/yolov5n_bs1.om    # 出现这个文件说明转换完成
```

`.onnx` 和 `.om` 不进 git（太大），所以模型要用 `scp` 单独拷。

## 日常运行

终端 1（小车）：识别 + 发布话题

```bash
cd /home/racecar/src/my_racecar/camera_capture
bash start_vision.sh
```

终端 2（小车）：看话题

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 /home/racecar/src/my_racecar/camera_capture/light_listener.py
```

或者用 ROS 自带命令：

```bash
ros2 topic echo /traffic_state
ros2 topic hz /traffic_state
ros2 topic echo /traffic_light_box
```

`Ctrl+C` 停止 `start_vision.sh`，会同时停掉识别。识别的日志在 `infer.log`。

额外参数会传给 `light_infer.py`，例如：

```bash
bash start_vision.sh --conf-box 0.4 --conf-lamp 0.4
```

## 只调识别、不发话题

```bash
bash run.sh              # 终端看状态
bash run.sh show         # 弹窗（需要 MobaXterm X server，很慢）
bash run.sh video        # 录 30 秒带框视频到 /tmp/out.mp4
```

## 注意

- 运行前关掉别的占用摄像头的程序（`record_video.py`、课程的 `video.py` 等），`start_vision.sh` 会检查。
- 在 `(base)` 终端里：识别用 `python3`（conda），ROS 节点用 `/usr/bin/python3`，脚本里已经分好，手动运行时别搞混。
- 电脑也想收到话题的话，电脑和小车要在同一网络、同一个 `ROS_DOMAIN_ID`。
- 还没接到停车控制（`/traffic_light` 已关闭）。改成视觉触发停车是下一步，那时再打开 `[控制]` 的几行。
