# AKE Drone Sim（ROS2 Humble）

一个三维四旋翼仿真工作区。系统包含刚体动力学、模拟 IMU/GPS、ESKF、SE(3) 控制器、三维 voxel map、前向 3D LiDAR、3D A*、B-spline/滚动预测参考、RViz2 和评测脚本。

本项目参考 `IntelligentUAVChampionshipBase/basic_dev/src` 中三个 ROS1 模块的算法结构，但所有节点均使用 ROS2 Humble、`rclcpp`、`ament_cmake` 和 `colcon` 重新实现：

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
├── launch/                   # sim.launch.py、open.launch.py
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

cd /home/ake/sim_drone/ake_drone_sim
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
source /home/ake/sim_drone/ake_drone_sim/install/setup.bash
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
3. 旋转矩阵反对称误差和角速度误差生成三轴力矩；
4. X 型分配矩阵把总推力/力矩转换为四电机 `omega²` 和 RPM；
5. 对速度、加速度、倾角、总推力和 RPM 做限幅。

## 3D voxel map 和模拟 LiDAR

地图内部是连续存储的三维占用数组，不创建任何 2D OccupancyGrid。默认范围为：

- x：`[-2, 10] m`
- y：`[-4, 4] m`
- z：`[0, 5] m`
- 分辨率：`0.2 m`

`/map/obstacles` 发布所有占用体素中心的 `PointCloud2`。默认场景含 5 个三维盒体，位于起点到 `(8,0,1.5)` 之间，并包含高障碍、立柱和顶置障碍。

模拟 LiDAR 默认为 120° 水平、60° 垂直、121×31 线束、10 Hz、8 m 量程。每条射线在 voxel grid 中步进，只返回第一个命中体素，因而支持遮挡。输出点云位于 `lidar_link`；规划器根据融合位姿转换到 `map`。

## 规划和失败条件

全局规划采用膨胀后的 3D voxel map、26 邻域 A*、欧氏启发和移动线段碰撞检查。安全距离默认为 0.5 m，无人机半径默认为 0.18 m，因此搜索占用层总膨胀为 0.68 m。

A* 路径先做可见性简化，再构造三次均匀 B-spline。曲线逐点碰撞检查；若平滑曲线不安全，则增加控制点密度；仍不安全时使用碰撞检查通过的 A* 折线局部目标回退，不会直接飞向原目标。滚动预测器限制速度、加速度和 jerk，并在发布前再次检查预测状态。

LiDAR 点云经过三帧历史缓存和 voxel 去重，用于近场轨迹威胁确认。连续多帧发现威胁时触发重规划。失败状态发布到 `/drone/planner_status`：

- 起点/目标越界或位于膨胀障碍物内；
- A* 无路径或超过搜索预算；
- 轨迹和回退参考均不安全；
- LiDAR 持续超时；
- 滚动预测结果进入膨胀体素。

失败时输出当前位置悬停参考。

## 实验和绘图

启动系统后记录单目标实验：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 2 1 1.5 --duration 20 --output /tmp/point.csv

ros2 run drone_visualization plot_results.py /tmp/point.csv --prefix /tmp/point
```

脚本生成位置和四电机 RPM 曲线。建议每个实验重新启动 launch，以清空动力学、融合器和轨迹状态。报告及已验证结果见 [`report/REPORT.md`](report/REPORT.md)。

运行包含高度变化的正方形多目标任务：

```bash
ros2 run drone_visualization waypoint_mission.py --timeout 90
```

## 参数调整

主要参数位于 `config/sim.yaml`：

- 质量、惯量、臂长、推力/扭矩系数和电机时间常数；
- IMU/GPS 噪声和固定随机种子；
- voxel 分辨率、地图范围和 6×N 障碍物数组；
- LiDAR FOV、线束数、量程、噪声和漏检率；
- A* 搜索预算、安全距离、参考速度和巡航高度。

修改物理参数时必须同步确认 hover RPM、控制增益和最大 RPM。修改地图范围或分辨率时，应同步修改 map、LiDAR 和 planner 的网格参数。
