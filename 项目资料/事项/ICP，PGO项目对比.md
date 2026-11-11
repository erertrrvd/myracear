# ICP、PGO 与两套项目中的“校准球”对比

> 本文按 2026-09-27 本地源码核对。`my_racecar（复件）` 指 C 盘桌面的智能车源码；“另一套 SLAM 项目”指 `C:\Users\lzc2\Desktop\UR5E机械臂(伺服电机)\slam\src(1)\src`。这里只说明源码实现和 RViz 标记含义，不把代码存在等同于实车精度已验证。文中的本地文件链接指向被检查的源码快照；后续若换仓库或部署版本，应重新核对。

## 一、先看结论：几个相似词说的是不同工作

| 场景 | 比较的数据 | 要求解什么 | 后续作用 | RViz 中相关图形 |
| --- | --- | --- | --- | --- |
| `my_racecar` 旧起点 ICP | 当前 `/scan` 转出的二维点，与保存的参考扫描点 | 起跑时的 `dx、dy、dyaw` | 修正供路径跟踪使用的当前位姿 | 不能凭“黄绿球”认定是它的显示 |
| `my_racecar` 新锥桶起点校准 | 当前检测的锥桶中心，与 `cones_ref.csv` 的参考锥桶中心 | 本次停车相对标准停车的 `dx、dy、dyaw` | 判断是否可起跑并显示停车偏差 | 绿球是参考锥桶；蓝球是当前锥桶；黄球是变换后的当前锥桶 |
| 另一套 SLAM 项目的 localizer ICP | 当前实时点云，与事先保存的 `.pcd` 地图 | 地图坐标系中的完整三维位姿 | 重定位，修正地图系与 LIO 局部系的关系 | 不等同于 `/pgo/loop_markers` 的球和线 |
| 另一套 SLAM 项目的 PGO 回环 | 当前关键帧点云，与历史关键帧构成的子图 | 回环两端的相对位姿，再优化整段轨迹 | 减轻累计漂移，让轨迹和地图更一致 | 金黄色球标关键帧位置；绿色线连回环两端 |

**最容易混淆的地方：** 智能车的黄球与绿球是“当前锥桶变换后的位置”和“标准锥桶位置”的对照；另一套 SLAM 项目的金黄色球与绿色线是“回环关键帧”和“回环边”。颜色相似不代表算法、对象或坐标系相同。

## 二、ICP 怎样匹配，为什么怕初值不好

ICP 的输入是源点云与目标点云，输出是将源点云对齐目标点云的旋转和平移。典型点到点流程是：用当前变换估计移动源点 → 在目标点中为源点寻找近邻对应 → 根据这些对应估计新的刚体变换 → 重复，直到变化足够小或达到迭代上限。近邻搜索可以通过 KD-Tree 实现；求解匹配点的刚体变换可以使用 SVD/Kabsch 等方法，具体取决于实现。

需要更准确地理解“ICP 迭代的是对应关系”：**对应关系和变换估计都会在循环中更新**。它不是枚举全局所有位姿；因此初值太远、对应距离门限过小、锥桶或货架几何重复、走廊缺少独特结构时，可能找不到足够正确对应，或收敛到错误的局部解。`hasConverged()` 为真只表示满足了当前算法的收敛条件，不自动证明到了正确地点。

可把“校准航向角”理解为：旋转估计中包含绕竖直轴的 yaw。若参考系认为车头方向与当前里程计方向不同，配准求出的旋转会体现这段差值。实际 ICP 往往同时估计平移；不是只测量一个 yaw，也不是修改 IMU 芯片的硬件标定。几何匹配的方向与雷达安装朝向、TF、时间戳和初始位姿有关。

## 三、`my_racecar`：两种起点校准要分开

### 3.1 旧流程：`get_local_path.cpp` 的二维 ICP

代码在 `start_icp` 开启时读取预存的参考雷达点；订阅 `/scan` 获取当前点，调用 `ICP2D::match(current_points_, ref_points_, 0, 0, 0)`。此处源码传入的初始 `dx、dy、yaw` 都是零，所以需要车辆初始停放与参考扫描足够接近。代码设置最多 30 次迭代、最大对应距离 0.4 m、最少 20 对对应，随后检查收敛与误差门限。成功后得到 `result_dx_、result_dy_、result_dyaw_`；在处理里程计位置时旋转和平移 `raw_x/raw_y`，并令 `current_yaw = raw_yaw + result_dyaw_`。这就是此旧流程中“ICP 校准航向角”的具体含义：**修正用于路径跟踪的位姿方向**，而非把雷达或 IMU 本体改标定。

源码：[get_local_path.cpp：加载参考与订阅雷达](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/get_local_path.cpp:1150>)；[ICP 调用与通过条件](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/get_local_path.cpp:1367>)；[位姿修正](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/get_local_path.cpp:1513>)。

### 3.2 新流程：`cone_calib_run.cpp` 的锥桶校准与黄绿球

先在标准起跑位置录制参考锥桶到 `cones_ref.csv`。再次放车时，节点从 `/scan` 累积雷达帧、检测当前锥桶候选，使用锥桶间成对距离寻找“哪个当前锥桶对应哪个参考锥桶”，然后用二维刚体闭式解得到 `dx、dy、dyaw`。源码说明这里采用二维 Kabsch 等价的解析解，**不是调用上一节的整帧点云 ICP，也不是 PGO 回环**。节点在取得足够帧后锁定本次结果，适合停车状态下检查。

`/cone_calib_run/markers` 中：

- **绿色大球 `R0..`**：参考文件中的标准锥桶中心位置。
- **蓝色球 `C0..`**：本次雷达检测且匹配成功的锥桶中心位置。
- **黄色球**：把对应蓝球按求出的旋转 `dyaw` 和平移 `dx、dy` 变换到参考坐标后得到的位置，理想情况下靠近对应绿球。
- **青色连线**：本次检测与参考锥桶之间的对应关系；旁边的毫米值表示变换后的点位残差。
- **下方绿色或红色状态球**：最终自检通过或失败。它是状态灯，与上方的绿色参考锥桶不是同一含义。

黄球贴近绿球说明本次几何拟合得较好，但**不等于允许起跑**。代码还同时检查平均匹配误差、最佳与次佳匹配的可区分度、位移量、转角及最大单点残差。要结合状态灯和终端结论；若锥桶布局对称、锥桶被移动或车停得太偏，可能被拒绝。该节点主要读取雷达、参考文件并发布可视化、打印判定，不发布车辆行驶命令。

源码：[cone_calib_run.cpp：流程说明](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/cone_calib_run.cpp:5>)；[二维位姿求解](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/cone_calib_run.cpp:510>)；[黄绿球构造](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/cone_calib_run.cpp:865>)；[通过条件](<C:/Users/lzc2/Desktop/my_racecar（复件）/src/cone_calib_run.cpp:1054>)；[RViz 预设说明](<C:/Users/lzc2/Desktop/my_racecar（复件）/rviz/cone_calib_run.rviz:9>)。

## 四、另一套 SLAM 项目：localizer ICP 与 PGO 回环

### 4.1 localizer ICP：把车重新定位到已知地图

`icp_localizer.cpp` 先从磁盘读 `.pcd` 地图并生成粗、精两种分辨率的目标点云；把当前点云作为源。它以提供的初始位姿 `guess` 先做粗 ICP，再用粗配准结果作精 ICP 的初值；每阶段都检查收敛与 fitness 阈值。成功后返回的是**完整三维位姿**，其中包含位置、roll、pitch、yaw，并非单独校准 yaw。`localizer_node.cpp` 将这个地图中位姿与当前 LIO 局部位姿比较，更新地图与局部系之间的偏移并发布 TF。地图可以存储在硬盘，实时帧来自正在运行的传感器。

例子：LIO 局部朝向估计是 10°，当前点云与已知地图对齐后得到地图朝向 25°，地图与局部系的方向关系就需要反映约 15° 差异。这只是说明坐标变换的例子，不代表当前实车观测值。

源码：[icp_localizer.cpp：读地图](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/localizer/src/localizers/icp_localizer.cpp:13>)；[粗精两阶段 ICP](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/localizer/src/localizers/icp_localizer.cpp:82>)；[localizer_node.cpp：更新偏移](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/localizer/src/localizer_node.cpp:245>)。

`yaw_fuser.py` 则是**另一条融合链**：其源码读取 LIO 的位置、车轮里程计的 yaw，并发布 `odom → base_footprint`。文件说明面向仿真 Gazebo 场景；它本身没有运行 ICP。不要因为名字含有 yaw 就把它当成“ICP 航向校准文件”。源码：[yaw_fuser.py](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/car_bringup/car_bringup/yaw_fuser.py:3>)。

### 4.2 PGO 回环：利用再次到达旧地点修正整条轨迹

PGO 是位姿图优化。关键帧位姿是节点；相邻关键帧的里程计相对运动是边。机器人走一圈，局部里程计误差累积后，实际回到起点却可能在估计轨迹中偏离起点。代码先用 KD-Tree 在**历史关键帧位置**中筛空间邻近候选，并要求时间相隔够久；随后拼接旧关键帧附近点云作为 ICP 目标子图，以当前关键帧为源，检验两处几何是否能对齐。这里第一层 KD-Tree 搜的是“候选关键帧位置”；它与 ICP 内部找点对应不是同一层工作。

若 ICP 收敛且得分满足阈值，程序求出两帧间的相对位姿并将其作为回环 `BetweenFactor` 加到图中。iSAM2 联合先验、里程计边和回环边调整**所有相关关键帧的全局位姿**，再更新局部轨迹到全局地图的偏移。作用是减轻轨迹首尾不合、墙面重影及地图整体变形；它不是给起跑点单独算一个 yaw，也不能保证错误回环不会拉歪地图。重复结构和 ICP 错配仍须现场核对。

源码：[simple_pgo.cpp：候选搜索](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgos/simple_pgo.cpp:137>)；[ICP 生成回环](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgos/simple_pgo.cpp:197>)；[因子图优化](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgos/simple_pgo.cpp:230>)。

### 4.3 为什么 RViz 把回环显示成金黄球与绿线

`pgo_node.cpp` 在 `/pgo/loop_markers` 发布 `MarkerArray`：金黄色 `SPHERE_LIST`（显示尺寸 0.3 m）放在回环两端的关键帧位置，绿色 `LINE_LIST` 把这两个位置连起来。这是**人为指定的可视化符号**，不是 ICP 算出球形地图，也不是把点云拟合成球；球只表示位置，不编码车头 yaw。绿色线表示被程序记录的一条回环关系，不能仅凭 RViz 图形证明它一定正确。源码：[pgo_node.cpp：Marker 类型、颜色和位置](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgo_node.cpp:169>)。

PGO 运行期间把历史关键帧点云和位姿留在内存中供回环搜索；按保存服务请求可写出 `map.pcd`，选择逐帧保存时还会写 `patches/*.pcd` 与 `poses.txt`。这个保存流程与 RViz 球体显示无关；源码能证明具备保存路径，不能证明当前设备已经执行保存。[关键帧入内存](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgos/simple_pgo.cpp:88>)；[保存服务](<C:/Users/lzc2/Desktop/UR5E机械臂(伺服电机)/slam/src(1)/src/pgo/src/pgo_node.cpp:282>)。

## 五、看 RViz 时快速分辨

1. 看显示项订阅的话题：`/cone_calib_run/markers` 对应智能车起跑锥桶校准；`/pgo/loop_markers` 对应另一套项目的回环标记。仅凭颜色不足以判断。
2. 看图形组合：**绿球 + 蓝球 + 黄球**是在比较标准锥桶、当前锥桶、变换后的当前锥桶；**金黄球 + 绿色连线**是在展示历史关键帧及其回环边。
3. 看任务发生的时间：起点校准在发车前停车状态下比较基准；重定位是车对齐已知地图；PGO 回环发生在行驶后再次接近旧地点时。
4. 看结果是否真正生效：视觉上接近只是线索，应同时核对节点日志、配准得分、通过条件、TF、实际路线或地图连续性。源码检查不代替实车检查。

一句话记忆：**智能车黄绿球是在校准起跑点；localizer ICP 是把实时车位姿对齐已知地图；PGO 回环是把“再次来到旧地点”变成约束，回头修正累计漂移。**
