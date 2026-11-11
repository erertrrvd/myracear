import os,glob,shutil,zipfile
from collections import Counter
SRC=r"D:\light_ai\data\imagesbiaozhu_merged";TMP=r"D:\light_ai\export\light_dataset";ZIP=r"D:\light_ai\export\light_dataset.zip"
VAL={'105134','112544','112926'}          # 整段视频留作验证：红灯 / 红变绿 / 绿灯
if os.path.exists(TMP): shutil.rmtree(TMP)
n=Counter();cls={'train':Counter(),'val':Counter()}
for f in sorted(glob.glob(os.path.join(SRC,'images','*.jpg'))):
    s=os.path.basename(f)[:-4];sp='val' if s[:6] in VAL else 'train'
    for k,ext in (('images','.jpg'),('labels','.txt')):
        d=os.path.join(TMP,k,sp);os.makedirs(d,exist_ok=True);shutil.copyfile(os.path.join(SRC,k,s+ext),os.path.join(d,s+ext))
    n[sp]+=1
    for l in open(os.path.join(SRC,'labels',s+'.txt')):
        if l.strip(): cls[sp][l.split()[0]]+=1
open(os.path.join(TMP,'data.yaml'),'w',encoding='utf8').write("# 不写 path：ultralytics 会以本文件所在目录为数据集根目录，搬到哪台电脑都能用\ntrain: images/train\nval: images/val\nnc: 3\nnames:\n  0: light_box\n  1: red_on\n  2: green_on\n")
open(os.path.join(TMP,'TRAIN.md'),'w',encoding='utf8').write(f"""# 红绿灯检测数据集（3 类）

类别：0 light_box（整个灯箱）、1 red_on（亮着的红灯）、2 green_on（亮着的绿灯）
格式：YOLO（每张图一个同名 .txt：类别 中心x 中心y 宽 高，已归一化）。空 .txt = 画面里没有灯箱（负样本）。
划分：按视频划分（验证集是完全没参与训练的视频）。训练 {n['train']} 张，验证 {n['val']} 张。
框数量 训练：{dict(sorted(cls['train'].items()))}  验证：{dict(sorted(cls['val'].items()))}

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
""")
if os.path.exists(ZIP): os.remove(ZIP)
with zipfile.ZipFile(ZIP,'w',zipfile.ZIP_STORED) as z:     # jpg 已压缩，不再压缩，打包快
    for root,_,fs in os.walk(TMP):
        for f in fs:
            p=os.path.join(root,f);z.write(p,os.path.join('light_dataset',os.path.relpath(p,TMP)))
print('train',n['train'],dict(cls['train']),'| val',n['val'],dict(cls['val']))
print('zip MB',round(os.path.getsize(ZIP)/1e6,1))
