# 上传到小车：改动清单与命令（复制即用，按顺序）

## 先搞清楚：哪些要传小车，哪些不用
| 改动 | 在哪台机器运行 | 要传到小车吗 |
|---|---|---|
| `distributed_mapping_vehicle.launch.py`（不再启动 `odom_tf_adapter`） | **小车** | **要**（本目录有副本）。你已经在车上手工改过同样的内容，功能一致，传上去只是让车上文件和项目一致 |
| `Run_car_safe.sh`（启动前清残留、退出时清整组进程） | **小车** | **要**（本目录有副本，车上还没有） |
| `lslidar_launch.py` 去掉小车 rviz | 小车 | 已在车上改好（有备份 `/home/racecar/_bak_rviz/`），**不用再传** |
| `map_autosave`、`scan_tf_bridge`、`mapping_status_logger`、`mapper.yaml`、两个电脑端 launch、`run_slam_windows.ps1`、`view_*` | **电脑** | **不用**。它们跑在电脑上。Windows 电脑直接用项目目录里的；Linux 电脑把项目同步过去再 `colcon build` 即可 |

所以真正要上车的只有两个文件。

## 一、在你的电脑上上传（PowerShell，要输入小车 SSH 密码）
```powershell
$car = "root@192.168.5.100"
scp "C:\Users\lzc2\Desktop\my_racecar（复件）\scripts\Run_car_safe.sh" "${car}:/home/racecar/Run_car_safe.sh"
scp "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\launch\distributed_mapping_vehicle.launch.py" "${car}:/home/racecar/distributed_mapping_vehicle.launch.py.new"
```
第二个文件先传成 `.new`，在车上确认后再放到位，避免直接覆盖。

## 二、在小车上安装（SSH 终端；可重复执行，自动备份）
```bash
mkdir -p /home/racecar/_bak_upload
# 1. 安全版启动脚本（不覆盖原来的 Run_car.sh）
sed -i 's/\r$//' /home/racecar/Run_car_safe.sh
chmod +x /home/racecar/Run_car_safe.sh
bash -n /home/racecar/Run_car_safe.sh && echo "Run_car_safe.sh 语法 OK"

# 2. 车端 launch：src 和 install 里的各份都更新（先备份）
NEW=/home/racecar/distributed_mapping_vehicle.launch.py.new
if [ -f "$NEW" ]; then
  /usr/bin/python3 -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read()); print('新 launch 语法 OK')" "$NEW" || exit 1
  for F in $(find /home/racecar/install/my_racecar /home/racecar/src/my_racecar -name distributed_mapping_vehicle.launch.py 2>/dev/null | xargs -I{} readlink -f {} | sort -u); do
    B=/home/racecar/_bak_upload/$(echo "$F" | tr '/' '_')
    [ -f "$B" ] || cp "$F" "$B"
    cp "$NEW" "$F" && echo "已更新: $F"
  done
fi
```

## 三、先试用（别急着替换原来的启动方式）
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
bash /home/racecar/Run_car_safe.sh
```
等驱动起来后**只按一次 `Ctrl+C`，多等几秒**，看到"所有进程已终止"，再在另一个窗口检查没有残留：
```bash
echo "=== 应该没有任何输出 ==="
pgrep -ax talker; pgrep -af lslidar_driver_node; pgrep -af encoder_node
```
没有输出就说明清理成功；再启动一次，`ros2 node list | grep -c IMU_publisher` 应为 `1`。

> `Run_car_safe.sh` 的“进程组清理”逻辑我只在本机做过语法检查，Windows 的 Git Bash 无法模拟 Linux 进程组，**没有在小车上实测过**，请按上面的步骤先试一次。

## 四、日常启动顺序（小车）
1. SSH 窗口一：`bash /home/racecar/Run_car_safe.sh`（取代 `bash Run_car.sh`）
2. SSH 窗口二：`ros2 launch my_racecar distributed_mapping_vehicle.launch.py`（只启动 `imu_encoder_mix`，**不要**加 `start_odom:=false`）
3. 电脑上：`.\run_slam_windows.bat -Check`，再 `.\run_slam_windows.bat`。建完图 `-Save`，看到"已完整保存"再关窗口。

## 五、回滚
```bash
cd /home/racecar
# 车端 launch：从备份恢复
for B in /home/racecar/_bak_upload/*distributed_mapping_vehicle*; do echo "备份: $B"; done
# 停用安全版，继续用原来的 Run_car.sh：直接 bash Run_car.sh 即可（原文件没动）
```
`_bak_upload` 里的文件名是把路径里的 `/` 换成 `_`，恢复时按文件名对应的路径手动 `cp` 回去。

## 本次改动内容（给你对照）
**小车端**
- `Run_car_safe.sh`：启动前若发现上次残留的 `hipnuc_imu/talker`、`lslidar_driver_node`、`encoder_node` 就先清掉；已有 `Run_car` 在运行则拒绝重复启动；`Ctrl+C` 时先正常收尾，再对整个进程组扫尾强杀，15 秒看门狗兜底。
- `distributed_mapping_vehicle.launch.py`：只启动 `imu_encoder_mix`，不再启动 `odom_tf_adapter`（电脑端自己生成 TF，省约 14% CPU）。

**电脑端（不用上车）**
- `map_autosave`（新增）：建图时每 60 秒自动存一份到 `<地图目录>\autosave\`（图片+位姿图），忘了保存也不会全丢；`-AutoSave 秒数`，`0` 关闭。正式保存仍用 `-Save`。
- `scan_tf_bridge`：开了里程计却收不到 `/encoder_imu_odom` 时打出明确错误（此时位姿恒为原点，地图几乎不增长——你 13:14 那趟就是这样）。
- `mapping_status_logger`：状态日志加 `known_cells`；地图 30 秒不增长写 `stalled` 事件。
- `run_slam_windows.ps1`：建图时前台每 15 秒打印进度；没里程计/地图不增长会红字/黄字告警；退出时检查有没有正式保存，没有就提示 `autosave` 位置；`-Check` 增加重复节点检测和 `/scan`、`/encoder_imu_odom` 频率。
- `mapper.yaml`：`occupancy_threshold 0.35→0.20`、`min_pass_through 4→3`（见下）。
- 两个电脑端 launch：新增 `autosave_period`、`beams` 参数。
- `view_path_on_map` / `view_csv.ps1`：新增 `-Inflate N`，只影响显示，把障碍格加粗 N 格。

### 关于“锥桶小/丢失”的实验依据
用模拟小车（979 点雷达、细锥桶 + 走动物体）对比：
| 条件 | 参数 | 锥桶找回 | 锥桶占据格 | 走动物体残留 |
|---|---|---|---|---|
| 车速 0.5 m/s，无畸变 | 0.35 / 4（原） | 6/6 | 39 | 0 |
| | 0.20 / 3 | 6/6 | 56 | 0 |
| | 0.10 / 2 | 6/6 | 73 | 0 |
| 车速 1.5 m/s，带扫描畸变 | 0.35 / 4（原） | **2/12** | 6 | 0 |
| | 0.20 / 3 | 6/12 | 19 | 0 |
| | 0.10 / 2 | 7/12 | 24 | 3 |

结论：原阈值在高速下会把细锥桶一起过滤掉（与你地图现象吻合）；降到 0.20/3 能显著改善且不留走动物体痕迹；再松会开始留痕迹。**高速下仍找不全**，建图那一圈开慢一点（0.5 m/s 左右）效果最好。增加重采样点数（`beams`）和“去畸变”在实验里没有稳定收益，所以默认值没改。
以上是模拟结果，**实车效果需要你再建一张图验证**。
