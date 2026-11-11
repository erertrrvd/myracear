# 在 Linux 电脑上把 yolov5n.onnx 转成小车用的 .om

适用：你训练 YOLOv5n 的那台 Linux 电脑（x86_64，Ubuntu）。显卡用不上，转换只用 CPU。

## 0. 先知道的几件事

- 小车上的 CANN 是 **7.0.RC1**（`/usr/local/Ascend/ascend-toolkit/7.0.RC1`），芯片 **310B**，对应转换参数 `--soc_version=Ascend310B4`。
  车上已确认的版本信息（`ascend_toolkit_install.info`）：

  ```
  version=7.0.RC1
  innerversion=V100R001C13SPC005B246
  compatible_version=[V100R001C29],[V100R001C30],[V100R001C13],[V100R003C10],[V100R003C11]
  arch=aarch64
  ```
- Linux 电脑上装的 CANN toolkit **必须也是 7.0.RC1**，并且是 **x86_64** 版。版本不一致，转出来的 `.om` 在车上可能加载失败。
- 转换只要做一次，产物是一个 `.om` 文件，拷到车上就能用。
- 要转的 ONNX 就是训练导出的 `best.onnx`（opset 12，输入 `images [1,3,640,640]`，输出 `[1,25200,8]`，已检查过）。

## 1. 准备 Python 依赖

CANN 7.0 需要 Python 3.7 到 3.10。先看版本：

```bash
python3 --version
```

如果是 3.11 以上，用 conda 建一个 3.9 的环境再继续：

```bash
conda create -n cann python=3.9 -y
conda activate cann
```

装依赖：

```bash
pip3 install attrs numpy decorator sympy cffi pyyaml pathlib2 psutil protobuf scipy requests absl-py
```

## 2. 下载并安装 CANN toolkit 7.0.RC1（x86_64）

从昇腾社区官网的“资源下载 → CANN 社区版”页面下载：

```
Ascend-cann-toolkit_7.0.RC1_linux-x86_64.run
```

约 1.3 GB。注意选 **7.0.RC1**、**linux-x86_64**、**toolkit**，不要选成 aarch64 或别的版本。

安装（普通用户会装到 `~/Ascend`，不需要 root）：

```bash
chmod +x Ascend-cann-toolkit_7.0.RC1_linux-x86_64.run
./Ascend-cann-toolkit_7.0.RC1_linux-x86_64.run --check
./Ascend-cann-toolkit_7.0.RC1_linux-x86_64.run --install
```

安装中出现协议提示时输入 `y`。

加载环境并确认 `atc` 能找到：

```bash
source ~/Ascend/ascend-toolkit/set_env.sh
which atc
cat ~/Ascend/ascend-toolkit/latest/x86_64-linux/ascend_toolkit_install.info
```

最后一条应显示 `version=7.0.RC1`，并对照车上的版本：

- `innerversion` 最好也是 `V100R001C13SPC005B246`，和车上完全一致最稳。
- 如果 `innerversion` 不同，但 `version=7.0.RC1`，一般也能用；转完后按第 5 节在车上测一下能不能加载，加载失败再换成完全一致的包。
- `arch` 在电脑上应是 `x86_64`（车上是 `aarch64`，这是正常的）。

如果是用 root 装的，路径是 `/usr/local/Ascend/ascend-toolkit/`，把上面的 `~/Ascend` 换掉。

## 3. 转换

把 `best.onnx` 放到一个英文路径的目录里，改名为 `yolov5n.onnx`：

```bash
mkdir -p ~/om_convert && cd ~/om_convert
cp /你的路径/yolov5/runs/train/exp/weights/best.onnx yolov5n.onnx
```

运行转换（参数和课程 `common/util/atc.sh` 完全一致）：

```bash
source ~/Ascend/ascend-toolkit/set_env.sh
atc --model=yolov5n.onnx \
    --framework=5 \
    --output=yolov5n_bs1 \
    --input_format=NCHW \
    --input_shape="images:1,3,640,640" \
    --log=error \
    --soc_version=Ascend310B4 \
    --input_fp16_nodes=images \
    --output_type=FP16
```

成功时最后会打印 `ATC run success`，目录里出现 `yolov5n_bs1.om`：

```bash
ls -lh yolov5n_bs1.om
```

一般几分钟内完成。

## 4. 拷到小车

```bash
ssh root@192.168.5.100 "mkdir -p /home/racecar/src/my_racecar/camera_capture/output"
scp yolov5n_bs1.om root@192.168.5.100:/home/racecar/src/my_racecar/camera_capture/output/
```

文件名和位置必须是 `camera_capture/output/yolov5n_bs1.om`，`start_vision.sh` 和 `run.sh` 按这个路径找。

## 5. 在小车上验证（root）

先确认模型能加载：

```bash
cd /home/racecar/src/my_racecar/camera_capture
source /usr/local/Ascend/ascend-toolkit/set_env.sh
python3 -c "from ais_bench.infer.interface import InferSession; s=InferSession(0,'output/yolov5n_bs1.om'); print('load ok')"
```

再跑识别和话题：

```bash
bash start_vision.sh
```

另开一个终端看话题：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /traffic_state
```

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| `atc: command not found` | 没有 `source ~/Ascend/ascend-toolkit/set_env.sh` |
| 安装时报 Python 版本或缺包 | 换 Python 3.9 环境，按第 1 节装依赖 |
| 转换报算子不支持 | 在训练电脑上用 `--opset 11` 重新导出 ONNX 再转 |
| 车上加载 `.om` 报版本不匹配 | Linux 电脑上的 CANN 版本和车上不一致，要装 7.0.RC1，最好 `innerversion` 也是 `V100R001C13SPC005B246` |
| 转换时路径报错 | 目录和文件名用纯英文 |
