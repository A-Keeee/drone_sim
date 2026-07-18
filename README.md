# AKE Drone Sim（ROS2 Humble）

一个三维四旋翼仿真工作区。系统包含刚体动力学、模拟 IMU/GPS、ESKF、SE(3) 控制器、三维 voxel map、前向 3D LiDAR、3D A*、B-spline/滚动预测参考、RViz2 和评测脚本。

- `pwm_se3_controller`：位置/速度误差、期望合力、期望姿态、SO(3) 姿态误差、力矩控制和 mixer；输出改为电机 RPM。
- `imu_gps_fusion`：IMU 预测、GPS 位姿更新、15 状态误差状态滤波。
- `AB_planner`：三维 A*、路径简化、三次 B-spline、弧长投影、受速度/加速度/jerk 限制的滚动预测和安全回退。

## 工作区结构

```text
ake_drone_sim/
├── README.md
├── src/
│   ├── drone_msgs/           # TrajectorySetpoint.msg
│   ├── drone_dynamics/       # 动力学、IMU/GPS、ESKF
│   ├── drone_controller/     # SE(3) 控制器与 RPM mixer
│   ├── drone_map/            # 3D voxel grid 与模拟 LiDAR
│   ├── drone_planner/        # 3D A*、B-spline、滚动预测规划器
│   └── drone_visualization/  # Marker、RViz 与 bringup 资源安装
├── launch/                   # sim、open、narrow、random 场景启动文件
├── config/                   # 物理、传感器、地图和规划参数
├── rviz/                     # RViz2 配置
├── scripts/                  # 数据记录与绘图
└── report/                   # 实验报告和结果
```

每个 `src` 子目录都是独立 ROS2 包，可以单独复用和构建。`drone_map` 导出 voxel 核心库，`drone_planner` 在其上实现搜索；`drone_msgs` 不依赖仿真实现。

## 环境与构建

目标环境：Ubuntu 22.04、ROS2 Humble、C++17。

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-pcl-ros \
  libeigen3-dev python3-matplotlib

cd ~/sim_drone/ake_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行测试：

```bash
colcon test
colcon test-result --verbose
```

## 启动

五障碍三维场景：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_visualization sim.launch.py
```

空旷目标点/悬停场景：

```bash
ros2 launch drone_visualization open.launch.py
```

强制爬升越过墙体的三维绕行场景：

```bash
ros2 launch drone_visualization narrow.launch.py
```

可复现随机三维场景（推荐用于观察完整规划链路）：

```bash
ros2 launch drone_visualization random.launch.py
```

另开终端发布穿越地图的目标：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

随机场景默认用种子 `20260715` 生成 14 个盒体，其中 60% 偏置在起点到目标点的直线路径附近，因此能明显看到红色 A* 路径绕行、青色 B-spline 平滑路径、橙色 MPC 预测和绿色实际轨迹。保持种子不变则每次启动完全一致；临时切换地图可直接运行 `ros2 launch drone_visualization random.launch.py seed:=42`，也可以修改 [`config/random.yaml`](config/random.yaml) 中的生成参数。

无界面运行时增加 `rviz:=false`。命令行发布目标：

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

悬停
```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

普通目标点
```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

五障碍避障
```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```


RViz2 的 Goal 工具已经配置到 `/drone/goal`；二维点击产生的零高度会被规划器替换为默认巡航高度 1.5 m。

### RViz2 规划路径图例

展开 RViz2 左侧 `Planning` 分组，可以看到：

- 红色细线 `Global A-star Path`：完整静态 voxel map 上的全局 3D A* 路径，对应 `/drone/planned_path`；
- 青色粗线 `B-spline / Safe A-star Fallback`：碰撞检查通过的 B-spline；如果 B-spline 不安全，则显示最终执行的安全 A* 折线，对应 `/drone/bspline_path`；
- 橙色粗短线 `Rolling MPC Prediction`：从无人机当前位置开始的局部滚动预测，对应 `/drone/mpc_prediction_path`；
- 绿色线 `Fused Flight Path`：融合里程计得到的实际历史轨迹，对应 `/drone/path`。

青色是全局可执行轨迹，会贯穿起点到终点；橙色只表示当前约 1.5 s 的局部预测，因此始终位于无人机附近并随飞行滚动更新。在 A* 安全折线回退模式下，青色路径会与红色路径大范围重合，这是正常现象；青色使用更粗的 Billboard 线覆盖显示。橙色预测现在在 B-spline 和 A* 回退两种模式下都会发布，并使用 transient-local QoS，因此重新打开 RViz2 后也能看到最后一次预测。

如果没有看到路径，先检查左侧 `Planning` 总开关及三个子项是否启用，再执行：

```bash
ros2 topic echo --once --qos-durability transient_local /drone/planned_path
ros2 topic echo --once --qos-durability transient_local /drone/bspline_path
ros2 topic echo --once --qos-durability transient_local /drone/mpc_prediction_path
```

三条消息的 `header.frame_id` 都应为 `map`，且 `poses` 不为空。只有发布新目标并完成规划后才会产生全局路径；悬停在初始点时路径很短，可能被无人机 Marker 遮挡。

## 节点和话题

| 包/节点 | 订阅 | 发布 |
|---|---|---|
| `drone_dynamics/quadrotor_dynamics_node` | `/drone/motor_rpm_cmd` | `/drone/ground_truth/odom`、`/drone/imu`、`/drone/gps_pose`、`/drone/motor_rpm`、真值路径 |
| `drone_dynamics/imu_gps_fusion_node` | IMU、GPS pose | `/drone/odom`、`/drone/path`、`map -> base_link` |
| `drone_controller/position_controller_node` | `/drone/odom`、`/drone/reference` | `/drone/motor_rpm_cmd` |
| `drone_map/voxel_map_node` | YAML 参数 | `/map/obstacles`、`/map/obstacle_markers` |
| `drone_map/simulated_lidar_node` | 完整 voxel map、真值姿态 | `/drone/lidar/points` |
| `drone_planner/ab_planner_node` | 融合里程计、目标、地图、LiDAR | 轨迹参考、安全目标、A*/B-spline/预测路径、状态 |
| `drone_visualization/visualization_node` | 里程计、目标、安全目标 | `/drone/markers` |

控制器和规划器只订阅融合后的 `/drone/odom`。真值只供传感器仿真、评测和对比，不进入控制反馈。

`drone_msgs/msg/TrajectorySetpoint` 包含位置、速度、加速度、yaw 和 yaw rate。电机命令为长度 4 的 `Float32MultiArray`，顺序为前左、后左、后右、前右。

## 坐标系和动力学

- 世界坐标系采用 ENU，`map` 的 z 轴向上。
- 四元数表示机体系到世界系旋转；推力沿机体正 z 轴。
- 电机实际转速满足一阶模型 `rpm_dot=(rpm_cmd-rpm)/tau`。
- 单电机推力 `F_i=k_F*omega_i^2`，反扭矩 `M_i=k_M*omega_i^2`。
- 平动：`m p_ddot = R[0,0,sum(F_i)] + F_ext - m g e3 - drag*v`。
- 转动：`I omega_dot = tau - omega × (I omega)`。
- 姿态通过机体系角速度积分并在每步归一化。

动力学和控制器共享相同的 X 型电机位置、旋转方向、质量、惯量、臂长、`k_F` 和 `k_M` 定义。地面模型包含垂向接触和起飞前静摩擦，避免未离地时穿地或侧翻。

## ESKF 与 SE(3) 控制

ESKF 名义状态为位置、速度、姿态、陀螺仪偏置和加速度计偏置。IMU 高频预测；低频 GPS pose 更新位置和姿态，协方差使用 Joseph 形式更新。模拟 IMU 姿态还用于抑制起飞初始化阶段的姿态漂移。

SE(3) 控制器计算：

1. 位置和速度误差生成期望加速度，并叠加轨迹加速度前馈；
2. 期望合力与 yaw 构造目标旋转矩阵；
3. 对相邻期望旋转矩阵差分并低通，得到完整 roll/pitch/yaw 期望角速度；
4. 旋转矩阵反对称误差和角速度误差生成三轴力矩；
5. X 型分配矩阵把总推力/力矩转换为四电机 `omega²` 和 RPM；
6. 对加速度、倾角、期望角速度、力矩、总推力和 RPM 做限幅。

默认位置环和姿态环采用接近临界阻尼的增益。控制器参数全部从 `sim.yaml` 读取，动力学参数必须与 `quadrotor_dynamics_node` 保持一致。3D A* 和平滑碰撞检查在后台线程运行，规划器主定时器在搜索期间持续发布锁定的悬停参考。里程计超过 `0.2 s` 未更新时停止输出；若仅规划参考超过 `2.5 s` 未更新而里程计仍正常，控制器会锁定当前位置悬停，不会在空中直接切为零 RPM。

## 3D voxel map 和模拟 LiDAR

地图内部是连续存储的三维占用数组，不创建任何 2D OccupancyGrid。默认范围为：

- x：`[-2, 10] m`
- y：`[-4, 4] m`
- z：`[0, 5] m`
- 分辨率：`0.2 m`

`/map/obstacles` 发布所有占用体素中心的 `PointCloud2`。默认场景含 5 个三维盒体，位于起点到 `(8,0,1.5)` 之间，并包含高障碍、立柱和顶置障碍。`random.launch.py` 还支持固定种子的随机盒体地图，可配置障碍数量、中心/尺寸范围、端点净空和直线路径附近的障碍比例。

模拟 LiDAR 默认为 120° 水平、60° 垂直、121×31 线束、10 Hz、8 m 量程。每条射线在 voxel grid 中步进，只返回第一个命中体素，因而支持遮挡。输出点云位于 `lidar_link`；规划器根据融合位姿转换到 `map`。

## 规划和失败条件

全局规划采用膨胀后的 3D voxel map、26 邻域 A*、欧氏启发和移动线段碰撞检查。安全距离默认为 0.5 m，无人机半径默认为 0.18 m，因此搜索占用层总膨胀为 0.68 m。

A* 路径先做可见性简化，再构造三次均匀 B-spline。曲线逐点碰撞检查；若平滑曲线不安全，则增加控制点密度；仍不安全时使用碰撞检查通过的 A* 折线局部目标回退，不会直接飞向原目标。B-spline 使用弧长表按 `reference_speed × dt` 采样，保证位置、速度和加速度参考时间一致。滚动预测器限制速度、加速度和 jerk，默认只选取约 `0.2 s` 前视状态，并在发布前再次检查预测状态。

进入目标点 `0.15 m` 且速度小于 `0.15 m/s` 后，规划器进入目标锁定状态，持续发布固定目标位置和零速度/零加速度，不再用变化的融合位置生成悬停参考。LiDAR 超时或规划失败时也只在进入悬停状态时锁定一次安全位置。

LiDAR 点云经过三帧历史缓存和 voxel 去重，用于近场轨迹威胁确认。连续多帧发现威胁时触发重规划。失败状态发布到 `/drone/planner_status`：

- 起点/目标越界或位于膨胀障碍物内；
- A* 无路径或超过搜索预算；
- 轨迹和回退参考均不安全；
- LiDAR 持续超时；
- 滚动预测结果进入膨胀体素。

失败时输出当前位置悬停参考。

## 实验和绘图

以下命令已在 Ubuntu 22.04、ROS2 Humble 上逐项实跑。`launch` 会持续占用终端，因此必须使用两个终端；每个新终端都要进入仓库并 source 环境：

```bash
cd ~/sim_drone/ake_drone_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可先确认三个脚本已经正确安装：

```bash
ros2 pkg executables drone_visualization | grep -E 'run_experiment|plot_results|waypoint_mission'
```

如果没有输出，重新执行 `colcon build --symlink-install` 并再次 `source install/setup.bash`。如果绘图提示缺少 Matplotlib，安装 `python3-matplotlib`。

### 1. 悬停和单目标

终端 A 启动空旷场景：

```bash
ros2 launch drone_visualization open.launch.py
```

终端 B 记录悬停：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 0 0 1.5 --duration 15 --output /tmp/hover.csv

ros2 run drone_visualization plot_results.py \
  /tmp/hover.csv --prefix /tmp/hover
```

若要在曲线中完整保留从地面起飞的阶段，先在终端 B 启动记录器，并在数秒内到终端 A 启动 `open.launch.py`。记录器会等待 DDS 数据；首个有效样本应为 `z=0.05 m` 左右。

单目标实验同样使用 `open.launch.py`。建议先停止并重新启动终端 A，再在终端 B 执行：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 2 1 1.5 --duration 20 --output /tmp/point.csv

ros2 run drone_visualization plot_results.py \
  /tmp/point.csv --prefix /tmp/point --goal 2 1 1.5
```

`--goal` 在新格式 CSV 中可以省略，因为 CSV 已保存每一行对应的目标；这里显式写出是为了使命令含义更清楚。

### 2. 多目标点

重新启动 `open.launch.py` 后，在终端 B 执行：


```bash
ros2 run drone_visualization waypoint_mission.py \
  --timeout 90 --output /tmp/waypoint.csv

ros2 run drone_visualization plot_results.py \
  /tmp/waypoint.csv --prefix /tmp/waypoint
```

任务依次执行 `(0,0,1.5) -> (2,0,1.5) -> (2,2,2.0) -> (0,2,1.5) -> (0,0,1.5)`。每个航点进入 `0.3 m` 并保持 `1 s` 后切换；全部到达返回 `0`，超时返回非零状态并指出完成了几个航点。

### 3. 五障碍静态避障

终端 A 重新启动五障碍场景：

```bash
ros2 launch drone_visualization sim.launch.py
```

终端 B 记录并按五障碍 AABB 计算净空：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 8 0 1.5 --duration 45 --output /tmp/avoidance.csv

ros2 run drone_visualization plot_results.py \
  /tmp/avoidance.csv --prefix /tmp/avoidance \
  --scenario five --safety-distance 0.5
```

### 4. 狭窄通道/明显三维绕行

终端 A 使用强制爬升场景：

```bash
ros2 launch drone_visualization narrow.launch.py
```

终端 B 执行：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 8 0 1.5 --duration 50 --output /tmp/narrow.csv

ros2 run drone_visualization plot_results.py \
  /tmp/narrow.csv --prefix /tmp/narrow \
  --scenario narrow --safety-distance 0.5
```

绘图器输出 `_position.png`、`_trajectory.png`、`_error.png`、`_rpm.png`、可用时的 `_attitude.png`、障碍场景的 `_clearance.png`，并把最终误差、到达时间、超调、稳态误差、路径长度、最大 RPM、饱和样本数和最小净空写入 `_metrics.txt`。实验前重新启动对应 launch，可以清空动力学、融合器、规划器和历史轨迹。

本机最新回归结果如下：

| 场景 | 结果摘要 |
|---|---|
| 从地面悬停 | 最终误差 `0.043 m`，最后 2 s 平均误差 `0.034 m`，无 RPM 饱和 |
| 单目标 `(2,1,1.5)` | 最终误差 `0.041 m`，`3.36 s` 首次进入 `0.3 m` |
| 多目标正方形 | 5/5 阶段到达，总用时约 `16.44 s`，最终误差 `0.067 m` |
| 五障碍 `(8,0,1.5)` | 最终误差 `0.063 m`，中心净空 `0.743 m`；扣除机体半径后净空 `0.563 m > 0.5 m` |
| 强制爬升绕行 | 最大高度 `4.121 m`，最终误差 `0.039 m`；扣除机体半径后净空 `0.758 m > 0.5 m` |

报告、曲线和完整指标见 [`report/REPORT.md`](report/REPORT.md) 与 [`report/results`](report/results)。

## 参数调整

主要参数位于 `config/sim.yaml`：

- 质量、惯量、臂长、推力/扭矩系数和电机时间常数；
- IMU/GPS 噪声和固定随机种子；
- voxel 分辨率、地图范围和 6×N 障碍物数组；
- 随机场景的障碍数量、固定种子、尺寸范围、端点净空和走廊偏置比例；
- LiDAR FOV、线束数、量程、噪声和漏检率；
- SE(3) 的 `kp/kv/kr/kw`、期望角速度滤波、力矩/倾角/加速度限幅和消息超时；
- A* 搜索预算、安全距离、参考速度、巡航高度、目标锁定阈值；
- 滚动预测的时域、步长、速度/加速度/jerk、增益和前视索引。

修改物理参数时必须同步确认 hover RPM、控制增益和最大 RPM。修改地图范围或分辨率时，应同步修改 map、LiDAR 和 planner 的网格参数。

当前调优基线为 `kp=[2.2,2.2,3.5]`、`kv=[3.0,3.0,3.8]`、`kr=[0.12,0.12,0.08]`、`kw=[0.065,0.065,0.065]`、参考速度 `0.8 m/s`。建议先在 `open.launch.py` 中调姿态阻尼和悬停，再进入障碍场景；不要先加入积分项掩盖参考不连续或姿态欠阻尼。
