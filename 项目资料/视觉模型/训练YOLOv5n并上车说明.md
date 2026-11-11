# YOLOv5n：训练 → 转 .om → 在小车上跑

> **进度（已完成）**：YOLOv5n 已训练完（100 轮，最好第 95 轮：精确率 0.999、召回率 1.000、mAP50 0.995），`best.onnx` 已检查通过（opset 12，输入 `images [1,3,640,640]`，输出 `[1,25200,8]`），并且已放进 `小车部署包\yolov5n.onnx`。用 `light_infer.py` 在电脑上跑 344 张验证图（红灯 172、绿灯 172），状态全部判对，约 25 fps（电脑 CPU，onnxruntime）。**下一步从第 3 节开始：拷到小车并转 .om。** 第 1、2 节已经做完，留着作记录。

整体流程：**训练电脑**训练 YOLOv5n 并导出 ONNX → **小车**把 ONNX 转成 `.om` → **小车**用摄像头跑 `light_infer.py`，你在车上看现象。

标 ⚠ 的是我没有验证过的地方，遇到报错先看第 6 节。

## 最短步骤（用部署包里的三个脚本）

电脑 PowerShell：

```bash
scp -r "D:\VSCode\ros2技术集\智能车总文件\视觉模型\小车部署包" root@192.168.5.100:/home/racecar/light_infer
```

小车（root）：

```bash
cd /home/racecar/light_infer
bash check_env.sh        # 只读检查环境，输出贴给我
bash convert.sh          # 后台转 .om，约 2 小时；tail -f convert.log 看进度
bash run.sh              # 转完后：终端看状态；run.sh show 弹窗；run.sh video 录带框视频
```

## 0. 先说明为什么不用 YOLO11

课程给小车的转换和推理脚本是按 **YOLOv5 v7.0** 写的（输出格式 `[1, 25200, 5+类别数]`）。YOLO11 输出格式不同，课程的后处理不能直接用，所以上车要用 YOLOv5n。YOLO11 模型只用来在电脑上验证数据。

## 1. 在训练电脑上训练 YOLOv5n（你做）

数据用 `数据\数据集打包_给训练电脑用\light_dataset.zip`（已划分训练 742 / 验证 344 张，3 类）。

```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
git checkout v7.0
pip install -r requirements.txt
pip install onnx
```

解压数据集，在 `yolov5` 目录里新建 `light.yaml`（路径改成你解压的位置）：

```yaml
path: D:/你解压的位置/light_dataset
train: images/train
val: images/val
names:
  0: light_box
  1: red_on
  2: green_on
```

训练（有 NVIDIA 显卡会很快）：

```bash
python train.py --img 640 --batch 16 --epochs 100 --data light.yaml --weights yolov5n.pt
```

导出 ONNX。**必须写 `--opset 12`**（v7.0 默认是 17，昇腾的转换工具可能不支持），**不要加 `--dynamic`**：

```bash
python export.py --weights runs/train/exp/weights/best.pt --include onnx --imgsz 640 --opset 12
```

产物：`runs/train/exp/weights/best.onnx`。**把 `best.pt`、`best.onnx` 和 `runs/train/exp/results.csv` 发回来。**

类别顺序（0 灯箱、1 亮红灯、2 亮绿灯）不能改。

## 2. 在电脑上检查 ONNX（我来做，不需要小车）

你把 `best.onnx` 放进 `模型\` 文件夹后告诉我。我会检查：输入名是不是 `images`、形状是不是 `[1,3,640,640]`、输出是不是 `[1,25200,8]`，并用验证图跑一遍，确认状态判断和 YOLO11 一致。这一步能排除“导出就错了”。

```bash
D:\light_ai\venv\Scripts\python.exe 小车部署包\light_infer.py --model 模型\best.onnx --source 数据\标注_合并3类\images
```

（这条需要先给 venv 装 `onnxruntime`，`pip install onnxruntime`。）

## 3. 把部署包拷到小车并转成 .om

### 3.1 拷贝（电脑 PowerShell，输入车的 SSH 密码）

```bash
scp -r "D:\VSCode\ros2技术集\智能车总文件\视觉模型\小车部署包" root@192.168.5.100:/home/racecar/light_infer
scp "D:\你放的位置\best.onnx" root@192.168.5.100:/home/racecar/light_infer/yolov5n.onnx
```

文件名必须叫 `yolov5n.onnx`，因为 `config.yaml` 里的 `weights: yolov5n` 决定了转换脚本去找哪个文件。

`小车部署包` 里有：`onnx2om.py`、`config.yaml`、`common/`（转换脚本）、`light_infer.py`（推理）、`det_utils.py`（课程自带的后处理）。bash 脚本已经转成 LF 换行。

### 3.2 转换（小车上，root）

```bash
cd /home/racecar/light_infer
source /usr/local/Ascend/ascend-toolkit/set_env.sh
which atc                      # 要能找到 atc，找不到说明 CANN 环境没加载
free -h                        # 看一下空闲内存
nohup python3 onnx2om.py > convert.log 2>&1 &
tail -f convert.log            # 看进度，Ctrl+C 只是停止查看，不会停止转换
```

- 课程记录板载转换要**约 2 小时**，而且有时会报错没有结果。转换期间别开别的重程序，板卡只有 4 GB 内存。⚠
- 成功后得到 `output/yolov5n_bs1.om`。**把它拷回电脑备份**到 `模型\`：

```bash
scp root@192.168.5.100:/home/racecar/light_infer/output/yolov5n_bs1.om "D:\VSCode\ros2技术集\智能车总文件\视觉模型\模型\"
```

- 转换失败的备选：在装了 CANN 7.0.RC1 x86 toolkit 的 Ubuntu 上转（`soc` 仍写 `Ascend310B4`）。toolkit 要登录昇腾社区下载，我没找到免登录地址。

## 4. 在小车上跑

先确认环境：

```bash
cd /home/racecar/light_infer
source /usr/local/Ascend/ascend-toolkit/set_env.sh
python3 -c "from ais_bench.infer.interface import InferSession; print('ais_bench ok')"
python3 -c "import torch, torchvision, cv2; print('torch ok', torch.__version__)"
```

用 **root 的 `python3`**（conda 的 3.9），不要用 `/usr/bin/python3`——ais_bench 和 torch 装在 conda 里。这和录视频时相反，别搞混。

跑之前**关掉别的占用摄像头的程序**（`ros2 run racecar video.py`、`record.py` 等）。

### 4.1 看现象（三选一）

终端文字（最快，推荐先用这个）：

```bash
python3 light_infer.py --model output/yolov5n_bs1.om
```

会打印状态变化和每 5 秒一次的帧率、平均推理耗时，例如：

```
[10:31:02] 状态 none -> red   STOP
  23.4 fps，平均推理 31 ms，状态 red
```

弹窗（需要 MobaXterm 的 X server；课程里测过，经 X11 显示很慢，看到的帧率不是真实帧率）：

```bash
python3 light_infer.py --model output/yolov5n_bs1.om --show
```

存带框视频再拷回电脑看（X11 太卡时用这个）：

```bash
python3 light_infer.py --model output/yolov5n_bs1.om --save-video /tmp/out.mp4 --max-seconds 30
scp root@192.168.5.100:/tmp/out.mp4 "D:\VSCode\ros2技术集\智能车总文件\视觉模型\数据\视频\"
```

### 4.2 状态含义

| 状态 | 含义 | 放行 |
|---|---|---|
| `red` | 灯箱里有亮红灯 | 停 |
| `green` | 灯箱里有亮绿灯 | **走** |
| `off` | 有灯箱，没亮灯 | 停 |
| `none` | 没看到灯箱 | 停 |
| `conflict` | 红绿同时亮（不正常） | 停 |

只有 `green` 才放行。连续 3 帧一致才切换状态。

### 4.3 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--conf-box` / `--conf-lamp` | 0.5 / 0.5 | 灯箱、亮灯置信度阈值 |
| `--confirm` | 3 | 连续几帧一致才切换 |
| `--ar-min` `--ar-max` | 2.5 / 4.5 | 灯箱长宽比范围（过滤椅子、人） |
| `--cy-min` `--cy-max` | 0.20 / 0.85 | 灯箱中心高度范围 |
| `--no-filter` | 关 | 关闭几何过滤 |
| `--source` | 0 | 摄像头编号 / 视频文件 / 图片文件夹 |
| `--state-file` | `/tmp/light_state.txt` | 每帧结果写到这里，给 ROS 节点读 |

### 4.4 离线跑一批图片（验证 .om 和电脑上结果一致）

先把几张验证图拷到车上，例如放在 `/home/racecar/light_infer/val/`：

```bash
python3 light_infer.py --model output/yolov5n_bs1.om --source val
```

每张图打印一行状态。把它和电脑上 `--model best.onnx` 的结果对比，应该一致。

## 5. 上车要测的清单

1. 红灯亮、绿灯亮、灯全灭、完全没有灯箱，状态各是否正确。
2. 对着椅子、人、深色衣服、红色物体，会不会出现 `red` / `green`（负样本很少，误检风险大）。
3. 帧率和耗时：目标 ≥ 15 fps。同时开一个终端看 `free -h` 和 `npu-smi info`。
4. **红灯判成绿灯必须为 0**（比赛提前通行罚 30 秒）。
5. 灯的位置变化：近、远、偏左偏右。

## 6. 可能遇到的问题 ⚠

训练电脑上：

- 报 `Unsupported global: models.yolo.Model`（`weights_only`）：新版 torch（2.6 起）的默认值变了，运行前设环境变量 `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`。
- 报 numpy 相关错误（`np.int`、`np.float` 等）：YOLOv5 v7.0 较老，装 `numpy<2`。
- `requirements.txt` 里某些包装不上：多半是版本太新或太旧，按报错逐个处理。

小车上：

- `atc: command not found`：没有 `source /usr/local/Ascend/ascend-toolkit/set_env.sh`。
- 转换报 `\r` 或 `bad interpreter`：脚本带了 Windows 换行。部署包里已经是 LF，如果你自己改过文件，用 `dos2unix` 处理。
- 转换报算子不支持：多半是 opset 不对，在训练电脑上用 `--opset 11` 重新导出再转。
- 转换中途被杀：多半是内存不够，关掉别的程序重试。
- `import ais_bench` 失败或推理报错：课程里 `pip install --upgrade torch torchvision` 解决过。
- 推理结果为空或全是错的框：检查 `.om` 是不是从**固定 640×640** 的 ONNX 转的，以及类别顺序是不是 0 灯箱、1 亮红灯、2 亮绿灯。
- `打不开 0`：摄像头被占用，或者 `/dev/video0` 不是这个摄像头。
- `--show` 没反应或报错：MobaXterm 的 X server 没开。脚本会自动关掉弹窗，继续在终端输出。

## 7. 第二阶段：接 ROS 和停车（还没做）

等第一阶段在车上跑通再做：

1. `light_bridge.py`（用 `/usr/bin/python3` 运行，ROS 的 Python）读 `/tmp/light_state.txt`，超过 0.5 秒没更新当作“没看到”，发布状态话题，并兼容现有的 `/traffic_light`（`1` = 绿灯放行）。
2. 在 `pure_track_test.cpp` 加状态机（行驶 → 接近 → 等红灯 → 放行 → 冷却），只有确认绿灯才放行。停车位置要用灯箱框的高度，需要现场实测。
3. 详见 `my_racecar（复件）\事项\红绿灯目标识别.md` 第 10 节。

## 8. 已知局限

- 训练数据来自同一间教室、同一块灯箱，上车跑通只代表流程通，不代表比赛可用。
- 负样本只有约 6 张，没有灯全灭和橙灯亮的数据。
- 需要补拍：灯全灭、不同距离角度光线、室外，以及 100 到 200 张没有灯箱的画面（椅子、人、桌子、锥桶、屏幕）。
