# 远程视觉：电脑识别，小车只传图

> 这是**独立于原来车上识别流程**的另一条路：原来的 `start_vision.sh`（车上 `.om` 识别）、`light_infer.py`、`light_bridge.py`、`run.sh` 一个字都没改，两条路可以随时切换。区别只是识别放在哪里。

## 概述

小车只负责采集和传图，识别放到 Windows 电脑，结果传回后由小车发布**同样的话题**（`/traffic_state`、`/traffic_light_box`，格式不变；`/traffic_light` 仍然关闭）。小车上不再需要 conda、CANN、torch、NPU 和 `.om`。

| 文件 | 在哪运行 | 作用 |
|---|---|---|
| `remote_vision_bridge.py` | 小车，`/usr/bin/python3` | 采集 + JPEG 编码 + TCP 服务端 + 发布话题 + 看门狗 |
| `start_remote_vision.sh` | 小车 | 检查摄像头占用后启动上面这个 |
| `pc_vision_client.py` | 电脑，`D:/light_ai/venv` | 连小车收画面 → 识别 → 把结果传回，本机窗口显示 |
| `remote_proto.py` | 两边都用 | 通信协议（标准库） |
| `start_pc_vision.ps1` | 电脑 | 电脑端入口：检查端口后启动 `pc_vision_client.py` |

电脑端直接复用 `light_infer.py` 的判断逻辑，所以要和 `light_infer.py`、`det_utils.py` 放在同一个目录里运行。

### 运行

小车（先关掉别的占用摄像头的程序，也不要同时运行 `start_vision.sh`）：

```bash
cd /home/racecar/src/my_racecar/camera_capture
bash start_remote_vision.sh
```

电脑 PowerShell（小车当服务端监听，电脑主动连，Windows 防火墙不会拦）。入口脚本会先检查端口通不通，不通会告诉你该查什么：

```bash
cd "C:\Users\lzc2\Desktop\my_racecar（复件）\camera_capture"
powershell -ExecutionPolicy Bypass -File .\start_pc_vision.ps1
```

等价的直接命令：`D:\light_ai\venv\Scripts\python.exe pc_vision_client.py --car 192.168.5.100`。

先在电脑上确认端口通：`Test-NetConnection 192.168.5.100 -Port 9000`。看话题的方法和以前一样（`ros2 topic echo /traffic_state`）。

### 行为

- **只发最新一帧**：电脑处理不过来时中间帧直接跳过，不排队，延迟不会越积越大。
- **延迟用小车自己的时钟算**（帧号对应拍摄时刻），不比较两台机器的时间。
- **看门狗**：没有电脑连接，或最新结果距拍摄超过 0.5 秒（`--stale-sec`），状态就是 `stale`；只有 `green` 才算放行。
- 电脑断线后自动重连；新连接会顶掉旧连接。
- 小车端每 5 秒打印采集帧率、编码耗时、码率、往返延迟 p50/p95。

### 测小车的负载

```bash
bash start_remote_vision.sh --selftest            # 只采集和编码，不连电脑，报告帧率、JPEG 大小和本进程 CPU 占用
top -b -n 1 | head -15                            # 对比：现在 start_vision.sh 运行时的负载
```

负载仍然太高时降低 `--fps`（默认 15）、`--quality`（默认 80）。注意模型输入是 640×640 的 letterbox，分辨率调小会降低识别精度。

### 注意

- **总延迟约 100 到 400 ms**（WiFi 单程 25 到 170 ms 加识别约 40 ms），车速 1 到 2 m/s 时会多滑 0.1 到 0.8 m，将来的停车阈值要把延迟算进去。
- 这条 TCP 端口没有认证，只在局域网内使用；建议 `BIND=192.168.5.100`。
- 比赛场地是否允许场外计算，规则里没有写，赛前要确认；链路断了只能退回车上 `start_vision.sh`（`.om`）或不放行。

## 和原来的车上识别怎么切换

两条路**互不影响**，原有文件都没改，只是同一时间只能开一条（都要用摄像头）：

| | 车上识别（原来） | 远程视觉（电脑识别） |
|---|---|---|
| 小车上运行 | `bash start_vision.sh` | `bash start_remote_vision.sh` |
| 电脑上运行 | 不需要 | `start_pc_vision.ps1` |
| 小车负载 | 高（NPU 推理加 torch/NMS） | 低（只采集编码和收发） |
| 依赖 | `.om`、CANN、conda | 电脑和 WiFi 在线 |

切换时先 `Ctrl+C` 停掉正在运行的那条；`start_remote_vision.sh` 发现车上识别在运行或摄像头被占用会拒绝启动并提示，`start_vision.sh` 本来就会检查摄像头占用。
