# yolov5n 电脑分布式通信和本地

红绿灯识别有两套，**不能同时运行**（共用摄像头 `/dev/video0` 和话题名）。

| | 本地 | 远程 |
|---|---|---|
| 识别在哪里 | 小车 NPU（`.om`） | 你的电脑 CPU（`.onnx`） |
| 小车负载 | 高 | 低（只拍摄、编码、转发） |
| 启动 | 小车：`bash start_vision.sh` | 小车：`bash start_remote_vision.sh`；电脑：`start_pc_vision.ps1` |
| 依赖 WiFi | 否 | 是（断线 0.5 秒变 `stale`） |
| 指令集 | `本地\指令集.md` | `远程\指令集.md` |

- 两套发布同样的话题：`/traffic_state`、`/traffic_light_box`。
- 切换前先 `Ctrl+C` 停掉当前这套，再用 `fuser -v /dev/video0` 确认摄像头空闲。
- 这里的文件是电脑项目 `camera_capture/` 的**副本**，改动请在项目里改。小车上的 `light_bridge.py`、`start_vision.sh` 有未同步的改动，见 `本地\指令集.md` 开头的提示。
- git 同步和冲突处理：`..\git和小车修改冲突解决方法\指令集.md`。
