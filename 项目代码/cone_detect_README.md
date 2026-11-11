# cone_detect 使用说明

四锥桶定位第一步：检测锥桶。**独立节点，不影响现有流程。**

---

## 一、编译

```bash
cd ~/your_ws
colcon build --packages-select my_racecar
source install/setup.bash
```

**已自动接入 CMakeLists，无需手动改。**

---

## 二、运行

```bash
# 基础运行（检测一次就停）
ros2 run my_racecar cone_detect

# 带 launch（可调参）
ros2 launch my_racecar cone_detect.launch.py

# 持续检测（在线调参时用）
ros2 launch my_racecar cone_detect.launch.py auto_reset:=true
```

---

## 三、RViz 观察 ★

```
Fixed Frame: laser_link

Add -> By topic -> /cone_detect/markers -> MarkerArray
```

### 图例

| 显示 | 含义 |
|---|---|
| 🟢 **绿色大球** | **选中的参考锥桶**（最近 N 个） |
| ⚫ **灰色小球** | **被淘汰的候选** ← 看到它就说明附近有多余锥桶，要警惕 |
| 🟡 **黄色线段** | 选中锥桶之间的两两连线（直观显示几何形状） |
| 🟡 **黄色数字** | 两两之间的距离 = **成对距离指纹** |
| ⚪ **白色文字** | 锥桶编号 + 到车的距离 |

### 关键观察点

```
✅ 只有 4 个绿球，没有灰球
   → 完美，起跑位附近恰好 4 个锥桶

⚠️ 有绿球也有灰球
   → 附近有多余锥桶，"取最近 4 个"可能选错
   → 需要改成按几何一致性挑选

❌ 绿球排成一条直线
   → 几何条件差，航向估计会不准
   → 终端会警告"几乎排成一条直线"

❌ 绿球数量 < 4
   → 检测参数需要调，或锥桶不在视野内
```

---

## 四、终端输出解读

```
================ 锥桶检测结果 ================
累积 20 帧，共 1287 个点，检出 4 个候选
id   选中  坐标(x, y)          距离(m)    宽度(m)   点数
0    ★    ( 1.234,  0.567)     1.358     0.142      18
1    ★    ( 3.421, -0.123)     3.423     0.156      24
2    ★    ( 4.876,  1.203)     5.022     0.131      15
3    ★    ( 2.109,  2.845)     3.542     0.168      21
---- 成对距离指纹（旋转平移不变量）----
  0-1 : 2.2481 m
  0-2 : 3.7305 m
  0-3 : 2.4598 m
  1-2 : 1.9752 m
  1-3 : 3.3001 m
  2-3 : 3.1904 m
几何分散度: 主方向散布 1.532 m, 次方向散布 0.821 m, 比值 3.5
    几何形状良好（分散），适合解位姿 ✓
==============================================
```

### 三个关键数字

| 输出 | 判读 |
|---|---|
| **宽度** | 应该在 0.10~0.25 之间。如果接近 `max_width`，说明参数偏紧 |
| **成对距离指纹** | ★ **跨次重启应该几乎一样（差 < 3cm）**，这是检测稳定性的唯一判据 |
| **几何分散度比值** | < 5 好；5~20 一般；**> 20 会警告"接近直线"** |

---

## 五、可调参数

| 参数 | 默认 | 怎么定 |
|---|---|---|
| `cluster_tol` | 0.25 | **最小锥桶间距的 1/3**。调太小→检出个数偏多（锥桶被拆开）；调太大→个数偏少（锥桶粘连） |
| `max_width` | 0.35 | ★ **量真实锥桶宽度 × 1.8**。锥桶底宽 25cm → 设 0.45；15cm → 设 0.27 |
| `min_points` | 4 | 远距离锥桶点数少，漏检就调小 |
| `accumulate_frames` | 20 | 越大越抗人群，但延迟增加。50Hz 雷达 20~50 都行 |
| `max_range` | 8.0 | 参考锥桶必须在 8 米内，否则被过滤 |
| `max_cones` | 4 | 取最近几个 |

**改了参数重启节点即可，不用重编译：**

```bash
ros2 run my_racecar cone_detect --ros-args \
    -p cluster_tol:=0.30 -p max_width:=0.45 -p accumulate_frames:=30
```

---

## 六、★ 验收实验（这一步是核心）

**成功标准不是"能看到锥桶"，而是"指纹稳定"。**

```
① 车放到起跑位，启动节点
② 抄下 6 个成对距离
③ 【不要动车】，杀掉节点，重新启动
④ 再抄一遍
⑤ 重复 5 次
⑥ 对比这 5 组数据
```

| 观察 | 说明 | 下一步 |
|---|---|---|
| 5 组差异 < 3cm | ✅ 检测稳定 | 进第 2 步（录参考锥桶） |
| 某几个锥桶时有时无 | ⚠️ 漏检 | ↑`accumulate_frames`，↓`min_points` |
| 距离抖动 > 10cm | ⚠️ 聚类不稳 | 调 `cluster_tol`；可能簇被拆分/合并 |
| 检出个数总 > 4 | ⚠️ 有干扰物 | 收紧 `max_width` 或 `max_range` |

---

## 七、诊断对照表

| 现象 | 原因 | 调什么 |
|---|---|---|
| **检出 7、8 个（比实际多）** | 一个锥桶被拆成多簇 | **↑`cluster_tol`** |
| **检出 1、2 个（比实际少）** | 相邻锥桶被粘成一簇 | **↓`cluster_tol`** |
| **人没被过滤掉** | `max_width` 太大 | **↓`max_width`** |
| **锥桶被当成噪声剔除** | `max_width` 太小 或 `min_points` 太高 | ↑`max_width`，↓`min_points` |
| **远处的锥桶检不到** | 点数太少 或 超量程 | ↑`accumulate_frames`，↓`min_points`，↑`max_range` |
| **一个都检不到** | 参数严重不匹配 | 先加大 `max_width` 和 `max_range` 排查 |

---

## 八、话题接口

| 话题 | 类型 | 内容 |
|---|---|---|
| `/cone_detect/result` | `std_msgs/Float64MultiArray` | `[n, x0,y0,r0,w0, x1,y1,r1,w1, ...]` |
| `/cone_detect/markers` | `visualization_msgs/MarkerArray` | RViz 可视化 |

**抓数据：**

```bash
ros2 topic echo /cone_detect/result --once
```

---

## 九、下一步（检测稳定之后）

```
第 2 步：录制参考锥桶
    在 path.csv 的坐标系下记录这几个锥桶的坐标 → cones_ref.csv

第 3 步：距离指纹匹配
    用 6 个成对距离确定"哪个观测点对应哪个参考点"
    （暴力试 24 种排列即可）

第 4 步：SVD 求解位姿
    复用 get_local_path.cpp 里 ICP2D::match() 的 SVD 代码，
    把"最近邻找对应"换成"指纹确定对应"

第 5 步：自检 + 明确反馈
    残差检查、界内检查、终端大字提示"可以起跑/重新摆放"
```

## 附：分布式 SLAM 建图（已移到单独文档）

这里原来是一份早期的建图草稿，已过时（会误导把电脑时间对齐到小车，而小车时钟本身是错的），已删除。
最新完整指令和问题记录见：

- `mapping/DISTRIBUTED.md`（本仓库）
- `/data/code/智能车/智能车(1)/智能车/文档/建图路线.md`
- `/data/code/智能车/智能车(1)/智能车/文档/slam建图问题解决.md`

电脑端工作空间是 `/data/code/智能车/智能车(1)/my_racecar（复件）(1)`，不再使用 `~/ros2_ws`。




在 Windows PowerShell 中，先按 Ctrl+C 退出当前的密码提示，然后逐行运行：
cd 'C:\Users\lzc2\Desktop\my_racecar（复件）'

git stash push -u -m "before merging car remote"
git fetch origin
git merge --no-edit origin/main
git push origin main
git stash pop --index
fetch 和 push 提示时，在终端输入小车的 SSH 密码。stash 是为了临时保管你已暂存的另一份文档；最后一条命令会恢复它。如果 fetch 失败或 merge 报冲突，就停在那一步，不要继续执行后面的命令，也不要使用 --force。
推送成功后，还需在小车终端更新实际运行的代码；推送到裸仓库本身不会更新 /home/racecar/src/my_racecar：
cd /home/racecar/src/my_racecar
git status --short --branch
git fetch origin
git merge origin/main
车上若报告未提交改动或合并冲突，也先停下处理，之后再编译。