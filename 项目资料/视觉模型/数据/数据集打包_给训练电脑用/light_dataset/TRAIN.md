# 红绿灯检测数据集（3 类）

类别：0 light_box（整个灯箱）、1 red_on（亮着的红灯）、2 green_on（亮着的绿灯）
格式：YOLO（每张图一个同名 .txt：类别 中心x 中心y 宽 高，已归一化）。空 .txt = 画面里没有灯箱（负样本）。
划分：按视频划分（验证集是完全没参与训练的视频）。训练 742 张，验证 344 张。
框数量 训练：{'0': 737, '1': 313, '2': 339}  验证：{'0': 344, '1': 172, '2': 172}

## 训练（YOLO11n）
1. 装环境：  pip install ultralytics      （有 NVIDIA 显卡请先按 pytorch.org 装 CUDA 版 torch）
2. 解压本 zip，进入解压后的目录，运行：
   yolo detect train model=yolo11n.pt data=data.yaml epochs=100 imgsz=640 batch=16 patience=20
   （第一次运行会自动下载 yolo11n.pt，需要联网；没显卡就加 device=cpu）
3. 结果在 runs/detect/train/weights/best.pt，同目录 results.csv 是训练曲线。
4. 把 best.pt（和 results.csv）发回来。

## 注意
- 类别顺序不要改：0 light_box，1 red_on，2 green_on。
- 数据来自同一间教室、同一块灯箱，验证分数会偏高，不代表换环境同样准。
- 负样本很少（约 6 张），误检（椅子、人被当成灯箱）还没解决，需要补拍。
