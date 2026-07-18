# AI 辅助编程使用说明

## 1. 使用声明与责任边界

本项目在代码审计、问题定位、局部代码修改、构建测试、日志与 CSV 分析以及文档整理过程中使用了 AI 辅助工具。

项目的以下内容由作者自行确定并承担最终责任：

- 项目话题、功能范围和六个 ROS2 软件包的总体划分；
- 动力学、状态估计、地图、规划、轨迹生成和安全降级方案的选型；
- 节点、Topic、消息类型、坐标系和 QoS 的接口设计；
- 控制器参考开源几何控制代码后的适配、重构和参数选择；
- 实验场景、验收指标、阈值和通过标准；
- 对源码、实际运行二进制、ROS2 实跑现象和实验数据的最终确认。

AI 主要承担辅助性工作，包括快速阅读代码、对照任务书审计仓库、提出排查思路、生成或修改局部代码、执行测试命令、整理实验数据以及协助编写文档。AI 的输出不直接作为项目完成依据，所有关键结论均经过作者检查，并通过编译、单元测试、ROS2 节点日志、Topic 检查、CSV、曲线或实际飞行现象验证。

---

## 2. 使用的 AI 工具

### 2.1 OpenAI Codex

本项目主要使用 **OpenAI Codex**，用途包括：

- 阅读 ROS2、C++17 和 Python 代码；
- 对照任务要求审计软件包、Launch、配置、测试和文档；
- 定位跨节点数据流和实时性问题；
- 协助修改规划器、控制器和实验脚本；
- 执行 `colcon build`、`colcon test` 等命令；
- 分析 ROS2 日志、CSV 和实验曲线；
- 整理 README、实验报告和本说明文件。

未使用 AI 直接替代 RViz 实际观察、实验标准制定、最终结果确认、Git 提交和演示视频录制。

---

## 3. Prompt 记录

仅保留重要的Prompt,前期多次对话重心为确立任务书，确立后一次性让codex搭建框架进行测试。

### 3.1 Prompt 1：项目任务书、仓库结构与 ROS1 算法迁移要求

```text
参考结构如下
ake_drone_sim/
  README.md
  src/
    drone_dynamics/
    drone_controller/
    drone_map/
    drone_planner/
    drone_visualization/
    drone_msgs/
  launch/
  config/
  report/

具体任务要求如下：
一、任务背景
使用 ROS2 和 AI 辅助编程/vibe coding，完成一个小型无人机仿真器的编程任务

二、任务目标
实现一个可运行的 ROS2 无人机仿真系统，至少包含：
1. 无人机动力学模块：输入为 4 个电机转速 RPM；
2. 无人机控制器模块：输入为目标点，输出为 4 个电机转速 RPM；
3. 地图模块：能够加载或生成障碍物地图；
4. 避障模块：无人机飞向目标点时需要考虑障碍物并生成安全路径或安全局部目标点；
5. 可视化：使用 RViz2；
6. 完整 README。

三、系统环境
1. Ubuntu 22.04；
2. ROS2 Humble；
3. C++ 或 Python 均可，推荐核心动力学和控制器使用 C++；
4. 可使用 Eigen、tf2、rviz2、rqt_plot、PlotJuggler 等工具；

四、系统功能要求

2. 动力学节点

实现 quadrotor_dynamics_node 或同等功能节点。

输入：
1. 4 个电机转速 RPM，例如 /drone/motor_rpm_cmd；
2. 建议使用 std_msgs/msg/Float32MultiArray 或自定义 MotorRPM.msg。

输出：
1. /drone/odom，类型建议为 nav_msgs/msg/Odometry；
2. /drone/imu，类型建议为 sensor_msgs/msg/Imu，可简化；
3. /tf，发布 map -> base_link；
4. /drone/path，类型建议为 nav_msgs/msg/Path。

动力学至少包含：
1. 状态量：位置、速度、姿态四元数、机体系角速度；
2. 电机一阶响应：实际转速逐渐逼近期望转速；
3. 推力模型：F_i = k_F * omega_i^2；
4. 力矩模型：根据 X 型四旋翼布局由 4 个电机推力和反扭矩产生 roll/pitch/yaw torque；
5. 平动方程：由姿态旋转后的总推力和重力更新速度与位置；
6. 转动方程：I * omega_dot = tau - omega x (I * omega)；
7. 四元数姿态积分与归一化；
8. 电机转速上下限保护。

9. 控制器节点

实现 position_controller_node 或同等功能节点。

输入：
1. 目标点 /drone/goal，类型可为 geometry_msgs/msg/PoseStamped；
2. 当前状态 /drone/odom。

输出：
1. /drone/motor_rpm_cmd，即 4 个电机 RPM。

控制器至少实现：
1. 位置误差到期望加速度的 PID 或 PD 控制；
2. 由期望加速度计算期望 roll、pitch、yaw；
3. 姿态或角速度控制，输出总推力和三轴力矩；
4. mixer：将总推力和三轴力矩分配为 4 个电机转速；
5. 推力、姿态角、RPM 的限幅；
6. 基本防炸机制，例如目标点过远时限速/限加速度。

7. 地图模块

实现 map_server_node、obstacle_map_node 或同等功能节点。

地图形式可任选一种：
1. 简单几何障碍物地图：若干球体、圆柱体、立方体或墙；
2. 栅格/体素地图：2D occupancy grid 或 3D voxel grid；
3. 点云地图：可参考 MARSIM 的点云地图思路，发布 sensor_msgs/msg/PointCloud2；
4. 程序随机生成地图，但必须支持固定随机种子，保证可复现实验。

地图模块至少输出：
1. /map/obstacles，可使用 MarkerArray、PointCloud2、OccupancyGrid 或自定义消息；
2. RViz2 中可见的障碍物；
3. README 中说明地图坐标系、障碍物尺寸和安全距离。

最低地图要求：至少包含 5 个静态障碍物，且障碍物必须位于起点到目标点之间，能迫使无人机改变路径。

5. 避障/规划模块

实现 local_planner_node、safe_goal_node 或同等功能节点。

输入：
1. 当前状态 /drone/odom；
2. 目标点 /drone/goal；
3. 地图或障碍物 /map/obstacles。

输出：
1. 给控制器的安全局部目标点、waypoint 或速度/加速度参考；
2. 建议 topic 为 /drone/reference、/drone/waypoints 或 /drone/safe_goal。

最低避障要求：
1. 无人机从起点飞到目标点时不得穿过障碍物；
2. 需要设置安全距离，例如 0.3 m 或 0.5 m；
3. 可视化中能看到规划路径或局部目标点；
4. 报告中必须说明避障算法的输入、输出、失败条件。

5. 可视化

RViz2 可视化：
1. 显示无人机模型或 Marker；
2. 显示目标点；
3. 显示历史轨迹；
4. 显示地图障碍物；
5. 显示规划路径、waypoints 或局部安全目标点；
6. 可以通过 RViz2 的 2D/3D Goal 或命令行发布目标点。

五、最低验收场景

请在 README 和报告中展示以下实验：
1. 悬停实验：无人机从地面或初始高度起飞并稳定在 (0, 0, 1.5) 附近；
2. 目标点实验：输入目标点 (2, 1, 1.5)，无人机能飞向目标并悬停；
3. 多目标点实验：按顺序飞到 3 到 4 个目标点，例如正方形航线；
4. 静态避障实验：地图中至少 5 个障碍物，无人机从起点飞到目标点并绕开障碍物；
5. 狭窄通道或绕行实验：设置一个需要明显绕行的场景，展示规划路径和实际轨迹；
6. 稳定性展示：给出位置误差曲线、RPM 曲线、轨迹图和最小障碍物距离曲线。

建议指标：
1. 最终位置误差；
2. 到达时间；
3. 最大超调量；
4. 稳态误差；
5. 最小障碍物距离；
6. 路径长度或飞行时间；
7. 是否出现姿态发散或 RPM 饱和。

最低要求：在合理参数下，目标点悬停误差应能收敛到 0.3 m 以内。
避障实验中，实际轨迹与障碍物的最小距离应大于你设定的安全距离。

六、加分项
1. 支持参数文件配置质量、惯量、电机系数、控制增益；
2. 支持风扰或外力扰动，并展示恢复能力；
3. 支持传感器噪声、IMU 简化模型；
4. 支持点云地图、体素地图或局部感知范围模拟；
5. 支持多无人机；
6. 支持轨迹输入，如圆轨迹、八字轨迹或 waypoint list；
7. 提供单元测试或脚本化自动评测；

原仓库路径中有三个重要参考模块：

1. basic_dev/src/pwm_se3_controller
  - 这是原 ROS1 SE3/PWM 控制器。
  - 新 ROS2 包中的控制器必须参考这个模块的控制思想：
    位置误差、速度误差、期望推力、期望姿态、姿态误差、力矩控制、mixer。
  - 但是不要保留 AirSim PWM 输出，不要使用 airsim_ros::RotorPWM。
  - 新控制器输出 /drone/motor_rpm_cmd，类型 std_msgs/msg/Float32MultiArray，长度 4。

2. basic_dev/src/imu_gps_fusion
  - 这是原 ROS1 IMU/GPS 融合模块。
  - 新 ROS2 包中需要实现 imu_gps_fusion_node。
  - quadrotor_dynamics_node 发布模拟 IMU 和模拟 GPS/真值观测。
  - imu_gps_fusion_node 订阅这些传感器数据，发布融合后的 /drone/odom。
  - 控制器只能订阅 /drone/odom，不直接使用 ground truth。

3. basic_dev/src/AB_planner
  - 这是原 ROS1 规划器模块。
  - 新 ROS2 包中实现 ab_planner_node 或 local_planner_node。
  - 参考原 AB_planner 的 A*、轨迹/waypoint、局部规划思想。
  - 新任务可以简化为 2D A* + 固定高度 + safe_goal 输出。
  - 规划器输入 /drone/odom、/drone/goal、/map/obstacles。
  - 规划器输出 /drone/safe_goal 和 /drone/planned_path。

协助我给出完整的项目任务书
```

### 3.2 Prompt 2：ROS2 三维无人机仿真与模拟 LiDAR 实施计划

```text
ROS2 三维无人机仿真与模拟 LiDAR 实施计划

总体方案

在 /home/ake/sim_drone/ake_drone_sim/src/ake_drone_sim 创建 ROS2 Humble 包 ake_drone_sim，完全本地部署，不使用 Docker、AirSim、Gazebo、ROS1 运行时或 2D 地图。

核心链路：

3D voxel map → 模拟前向 3D LiDAR → AB planner → 轨迹参考 → SE(3) 控制器 → 电机 RPM → 三维动力学 → IMU/GPS → ESKF

控制器、规划器和里程计按照原 ROS1 模块的算法逻辑重写为 ROS2；模拟 LiDAR 点云会真正参与局部碰撞检查、MPC 约束与重规划。

节点与公共接口

节点：quadrotor_dynamics_node
输入：/drone/motor_rpm_cmd
输出：/drone/ground_truth/odom、/drone/imu、/drone/gps_pose、/drone/motor_rpm、/drone/ground_truth/path

节点：imu_gps_fusion_node
输入：/drone/imu、/drone/gps_pose
输出：/drone/odom、/drone/path、TF map -> base_link

节点：position_controller_node
输入：/drone/odom、/drone/reference
输出：/drone/motor_rpm_cmd

节点：voxel_map_node
输入：YAML 参数
输出：/map/obstacles、/map/obstacle_markers

节点：simulated_lidar_node
输入：/map/obstacles、/drone/ground_truth/odom
输出：/drone/lidar/points

节点：ab_planner_node
输入：/drone/odom、/drone/goal、/map/obstacles、/drone/lidar/points
输出：/drone/reference、/drone/safe_goal、A*/B-spline/MPC 路径、规划状态

节点：visualization_node
输入：状态、目标、RPM
输出：无人机、目标、安全参考和雷达视场 Marker

新增 TrajectorySetpoint.msg，包含时间戳、位置、速度、加速度、yaw 和 yaw rate。四电机命令继续使用长度为 4 的 Float32MultiArray。

TF 结构：
- 融合节点发布 map -> base_link。
- 发布固定变换 base_link -> lidar_link。
- LiDAR 点云坐标系为 lidar_link。
- 动力学真值使用独立 ground_truth/base_link 辅助帧，不覆盖融合 TF。

模拟前向 3D LiDAR

- 默认模拟约 120° 水平、60° 垂直前向视场，10 Hz，最大量程 8 m。
- 水平和垂直分辨率、扫描频率、最小/最大量程、噪声、漏检率和随机种子均可配置。
- LiDAR 节点根据真值姿态进行传感器仿真，但只发布 lidar_link 坐标系中的测量点；规划器无法访问真值状态。
- 对三维 voxel map 使用 3D DDA/ray marching 射线遍历：
  - 每条射线只返回第一个命中体素，正确模拟遮挡；
  - 无命中射线不生成障碍点；
  - 命中距离加入可配置高斯噪声；
  - 输出字段至少包含 x/y/z，消息类型为 sensor_msgs/msg/PointCloud2。
- 点云使用 sensor-data QoS；完整地图使用 transient-local QoS。
- RViz2 同时显示完整地图、当前 LiDAR 点云和 LiDAR 视场边界，便于确认局部感知范围。

动力学、控制器与里程计

动力学与传感器

- 实现位置、速度、四元数、机体系角速度和四个实际 RPM 状态。
- 实现电机一阶响应、X 型四旋翼推力/反扭矩、三维平动和转动方程、四元数积分归一化、RPM 限幅、地面接触与数值异常保护。
- IMU 发布机体系角速度和 specific force；GPS 发布带噪声的低频 PoseWithCovarianceStamped。
- 所有物理参数、传感器噪声、偏置和固定随机种子由 YAML 配置。

ESKF 里程计

- 按原 imu_gps_fusion 重写位置、速度、姿态、陀螺仪偏置和加速度计偏置的误差状态滤波器。
- IMU 高频预测，GPS 位置与姿态低频更新，使用 Joseph 形式更新协方差。
- /drone/odom 和 map -> base_link 只由融合节点发布。
- 控制器和规划器只使用融合里程计，不订阅 ground truth。

SE(3) 控制器

- 按原 pwm_se3_controller 保留位置误差、速度误差、加速度前馈、期望合力、目标旋转矩阵、SO(3) 姿态误差、角速度误差、刚体补偿力矩和 mixer。
- 将 AirSim/NED 符号改为统一 ENU 约定，移除 PWM、RotorPWM 和 AirSim 服务。
- 使用与动力学一致的 X 型分配矩阵，将总推力和三轴力矩转换为四电机 RPM。
- 实现目标距离、速度、加速度、倾角、推力、力矩和 RPM 限幅，以及里程计/参考超时保护。

3D voxel map 与 AB planner

体素地图

- 内部使用连续一维数组保存真正的三维占用体素，不生成 OccupancyGrid。
- /map/obstacles 使用 PointCloud2 发布占用体素中心，供规划器全局 A*、LiDAR 仿真和 RViz2 使用。
- 支持 YAML 固定障碍物与固定种子的随机生成模式。
- 默认分辨率约 0.2 m，安全距离 0.5 m；固定场景至少含 5 个位于起点与目标之间的三维障碍物。
- 狭窄场景设置有限高度开口或高空绕行空间，迫使路径产生明显 z 方向变化。

原 AB planner 主链路

- 全局 3D A* 使用完整静态 voxel map；模拟 LiDAR 用于局部轨迹威胁判断和 MPC 障碍约束。
- A* 保留原代码的 26 邻域、欧氏启发、三维搜索窗口、搜索预算和父节点回溯思想。
- 对安全距离和无人机半径进行三维体素膨胀；检查邻接移动线段，禁止对角穿过体素棱角。
- 保留原路径优化中的数据项、平滑项、z 平滑、最大偏移和转弯约束。
- 保留三次均匀 B-spline、导数计算、最近点投影与弧长表跟踪。
- 保留 [p,v,a] 三重积分模型、jerk 输入、预测时域和速度/加速度/jerk 约束的 MPC，使用本地 ros-humble-osqp-vendor。
- B-spline 和 MPC 结果都执行三维碰撞采样；求解或碰撞检查失败时逐级回退到安全 B-spline 或限速 A* 参考。

LiDAR 接入规划器

- 规划器通过 tf2 将 /drone/lidar/points 从 lidar_link 转换到 map。
- 按原 AB planner 点云逻辑执行：
  - 点云有效期检查；
  - 固定大小历史帧缓存；
  - voxel 降采样和重复点合并；
  - 当前 B-spline 前向区间威胁检测；
  - 障碍确认/清除帧数滞回；
  - 最近障碍点筛选并转换为 MPC 局部约束。
- 当前轨迹与新 LiDAR 点云的距离低于安全阈值时立即触发重规划。
- 即使完整地图已有障碍物，MPC 的近场障碍集合仍以实时 LiDAR 点云为准，确保雷达消息实际影响规划结果。
- LiDAR 超时但全局地图有效时降低速度并继续全局路径；持续超时超过保护阈值则悬停并发布失败状态。
- 地图阻塞、目标越界、A* 超预算、B-spline 碰撞、MPC 求解失败和 LiDAR 长时间失联均通过 /drone/planner_status 明确报告。
- 不迁移赛事专用 TTC 扇区应急接管、掉头和 recovery-nudge 状态机。

本地部署与文档

- 提供 Ubuntu 22.04 + ROS2 Humble 的本地依赖安装说明，包括 Eigen3、RViz2、PCL/PointCloud2、osqp_vendor 和 Matplotlib。
- 使用 colcon build --symlink-install 构建，不创建 Docker 相关文件。
- 提供完整参数 YAML、ROS2 Python launch 和 RViz2 配置。
- README 说明坐标系、TF、体素结构、LiDAR 参数和射线模型、障碍尺寸、安全距离、SE(3)、ESKF、A*、B-spline、MPC、回退策略及所有运行命令。
- REPORT.md 使用自动实跑生成的真实指标、曲线和 RViz2 截图。

实验与测试

自动实验包括：
- (0,0,1.5) 悬停；
- (2,1,1.5) 单目标；
- 包含高度变化的 3～4 个多目标点；
- 至少 5 个障碍物的三维避障；
- 需要爬升或明显三维绕行的狭窄场景；
- LiDAR 遮挡、量程边界、点云短时丢失和持续失联测试。

新增 LiDAR 单元测试：
- 体素命中距离；
- 第一障碍物遮挡后方体素；
- 水平/垂直视场边界；
- 最大量程；
- 固定随机种子复现；
- 点云坐标系和时间戳；
- 规划器点云变换、历史合并、威胁检测与超时保护。

最终验收：
- 悬停和目标点误差收敛到 0.3 m 内；
- 实际轨迹到障碍物的最小距离严格大于 0.5 m；
- 规划器确实收到并使用 LiDAR 点云，遮挡物进入视场后能够触发局部约束或重规划；
- 无 NaN、四元数失效、姿态发散或持续 RPM 饱和；
- colcon test 和全部自动实验通过，并生成位置误差、RPM、三维轨迹和最小障碍距离图。

已确定假设

- 规划器采用“完整静态 voxel map 做全局规划 + 前向模拟 LiDAR 做局部安全校验”的结构。
- 模拟 LiDAR 默认采用原 AB planner 风格的前向有限视场，而不是 360° 全周扫描。
- 地图、规划和碰撞检查全部为三维；PointCloud2 只是体素地图和 LiDAR 的 ROS2 传输格式。
- 原 ROS1 仓库保持不变；算法迁移会保留控制、融合和规划逻辑，但重新实现 ROS2 通信、ENU 坐标约定和 RPM 输出。
```

### 3.3 Prompt 3：仓库完整性审计

```text
根据完整任务书检查当前仓库是否满足任务目标。
```


---

## 4. AI 协助完成的工作

### 4.1 仓库检查与需求映射

AI 协助把任务要求拆分为以下检查项：

- 四旋翼动力学；
- 电机模型和四电机 RPM 输入；
- 控制器及 mixer；
- IMU/GPS 仿真与 ESKF；
- 三维 voxel map 和模拟 LiDAR；
- 3D A*、路径平滑与滚动预测；
- RViz2 可视化；
- 单元测试和最低验收实验；

### 4.2 五障碍坠地问题定位

AI 协助读取控制器日志、规划器日志和 CSV，定位到以下因果链：

1. 规划器在主定时器回调中同步执行耗时 A* 和 B-spline 安全检查；
2. 规划期间 `/drone/reference` 停止更新；
3. 控制器检测到参考时间超过 `2.5 s`；
4. 原有故障处理直接输出四电机零 RPM；
5. 无人机在等待规划时失去推力并坠地。

该问题不能只通过最终位置误差发现，因为无人机在规划完成后可能重新起飞并到达目标。真正的异常由高度曲线、RPM 曲线和控制器日志共同确认。

### 4.3 异步规划安全修复

在作者确定“耗时规划不得阻塞安全参考发布”的原则后，AI 协助完成以下修改：

1. 定义 `AsyncPlanResult`，保存搜索状态、路径、B-spline、弧长表和可视化采样；
2. 启动规划前复制地图、起点、目标和 generation 快照；
3. 使用 `std::async(std::launch::async, ...)` 执行 A*、平滑和碰撞检查；
4. 主线程通过 `future.wait_for(0)` 非阻塞轮询结果；
5. 规划期间固定 `hold_position`，持续发布 `PLANNING_HOLD` 悬停参考；
6. 新目标到来时增加 generation，丢弃已被替代的旧规划结果；
7. 捕获后台异常并转换为 `PLAN_EXCEPTION`；
8. 规划未完成时避免 LiDAR 对旧 spline 触发无效重复规划。

### 4.4 控制器参考超时降级

原控制器将里程计失效和轨迹参考失效合并为单一 `fresh` 判断。AI 协助拆分为：

- `odom_fresh`；
- `reference_fresh`。

当 odom 新鲜而 reference 过期时，控制器：

- 保存当前位置作为 failsafe 位置参考；
- 将参考速度和加速度置零；
- 将 yaw rate 置零；
- 重置期望姿态差分状态；
- 继续通过正常控制律计算悬停所需 RPM。


### 4.5 实验执行与指标分析

实验场景、指标和通过标准由作者提出，AI 协助执行命令和整理数据，主要统计：

- 最终位置误差；
- 首次进入目标点 `0.3 m` 球的时间；
- 稳态窗口误差；
- 最低和最高高度；
- 相邻轨迹点累计路径长度；
- 四电机转速和 RPM 饱和样本数；
- 轨迹中心到原始障碍 AABB 的最小距离；
- 扣除无人机半径后的机体表面净空；
- 最大 roll/pitch 角；
- 规划等待阶段的高度与 RPM。


### 4.6 文档整理

AI 协助整理和修改：

- README 中的启动、记录、绘图和验收命令；
- 异步规划和参考超时悬停的设计说明；
- `report/REPORT.md` 中的架构、公式、参数、实验结果、失败案例和局限性；
- 本 `ai_usage.md`。

所有实验数值在写入文档前均由作者结合 CSV、metrics 文件和实际日志复核。

---

## 5. 自行确定、确认或修改的核心内容

### 5.1 项目主题、模块和算法选型

以下内容由作者确定，而不是由 AI 自动决定：

- 以 ROS2 四旋翼动力学、控制、状态估计和三维避障作为项目主题；
- 划分 `drone_msgs`、`drone_dynamics`、`drone_controller`、`drone_map`、`drone_planner` 和 `drone_visualization` 六个包；
- 使用四电机 RPM 作为动力学输入；
- 使用带前馈的 SE(3)/SO(3) 串级控制结构；
- 使用模拟 IMU/GPS 和 15 状态 ESKF；
- 使用三维 voxel map、前向模拟 LiDAR 和 26 邻域 3D A*；
- 使用可见性简化、三次 B-spline 和受速度、加速度、jerk 约束的滚动预测；
- 规划失败时采用悬停、A* 折线回退和 generation 丢弃旧结果等安全策略。

控制器设计参考了开源几何控制代码，但 ROS2 消息接口、参数、mixer、电机顺序、超时处理和本项目的安全降级逻辑由作者结合任务需求确认和调整。使用开源代码时保留相应来源说明，并遵守其许可证要求。

### 5.2 动力学公式与符号约定

作者重点确认了以下内容：

- 世界坐标系采用 ENU；
- 推力方向沿机体 `+z`；
- 四元数表示机体系到世界系的旋转；
- RPM 先乘 `2π/60` 转换为 `rad/s`，再进入平方推力模型；
- 四电机顺序为前左、后左、后右、前右；
- X 型布局的有效力臂为 `arm_length / sqrt(2)`；
- yaw 反扭矩符号采用 `+ - + -`；
- 动力学和控制器必须使用完全一致的分配矩阵；
- 四元数积分后必须归一化；
- 电机采用一阶响应，而不是瞬时到达命令 RPM。

### 5.3 控制器公式与接口


- 控制器订阅规划器输出的 `TrajectorySetpoint`，而不是直接订阅用户 goal；
- 外环同时使用位置误差、速度误差和加速度前馈；
- 期望姿态由期望合力方向和期望 yaw 构造；
- 姿态误差使用 SO(3) 反对称矩阵的 vee 映射；
- mixer 输出四个电机 RPM，而不是 PWM 或归一化油门；
- 三轴力矩、总推力、期望倾角和 RPM 均需要限幅；
- 里程计失效与轨迹参考超时必须采用不同的安全策略；
- 飞行过程中 reference timeout 不得直接输出零 RPM。

### 5.4 地图、规划与安全距离


- 地图采用三维 voxel grid，而不是二维 OccupancyGrid；
- 默认分辨率为 `0.2 m`；
- 任务要求的安全距离为 `0.5 m`；
- 无人机近似半径为 `0.18 m`；
- A* 对无人机中心使用约 `0.68 m` 的总膨胀；
- 路径线段和平滑曲线都必须进行碰撞检查；
- “中心轨迹到障碍距离”和“机体表面净空”是两种不同指标；
- 报告中机体表面净空应由中心距离减去无人机半径得到。

### 5.5 ROS2 Topic、Frame 与 QoS


- `/drone/motor_rpm_cmd` 数据长度必须为 4；
- `/drone/ground_truth/odom` 仅用于传感器仿真和评测，不能直接作为控制反馈；
- `/drone/odom` 由 ESKF 输出；
- 地图、路径和目标统一使用 `map` frame；
- A* 路径、B-spline/安全折线和滚动预测分别发布独立的 `nav_msgs/Path`；
- 静态地图和规划路径使用适合后启动节点接收的 Transient Local QoS；
- LiDAR 等传感器点云使用 SensorData QoS；
- `map -> base_link` 由状态估计节点发布。

### 5.6 测试标准

测试场景、评价方法和验收标准由作者提出，AI 负责辅助执行和统计。主要标准包括：

- 悬停和目标点最终误差小于 `0.3 m`；
- 多目标任务需要进入阈值并持续规定时间后才切换；
- 避障实验需同时满足最终到达、安全净空和飞行稳定性；
- 不能只看最终误差，还必须检查中途最低高度、RPM、姿态和异常日志；
- RPM 饱和、NaN、穿障和零 RPM 坠地均视为失败；
- 修改后必须进行正常场景回归和故障注入测试。

---

## 6. AI 产生过的问题及修正过程

### 6.1 早期实现将耗时规划放在主定时器线程

早期代码能够在规划结束后重新起飞并到达目标，因此只查看最终误差会误判为成功。CSV 显示规划等待阶段 RPM 降为零、无人机坠地，说明问题出在实时数据流而不是最终路径本身。

修正包括：

- 将 A* 和平滑检查移入后台；
- 主线程持续发布悬停参考；
- 控制器在 reference timeout 时保持当前位置；
- 增加暂停规划器的失败注入测试。

**经验：** 对闭环系统不能只检查终点，还要检查完整时间序列和中间安全状态。

### 6.2 指标脚本和文档命令曾存在场景口径问题

曾发现以下问题：

- 障碍物 AABB 在绘图脚本中按默认场景硬编码，不能直接用于 random 或 narrow 场景；
- 单目标绘图若未传 `--goal 2 1 1.5`，误差会按默认悬停目标计算；
- Launch 和记录器必须在不同终端运行，旧 README 没有充分说明；
- “安全距离”未明确区分中心距离和机体表面净空。

修正后，README 明确区分 `open`、`sim`、`narrow` 场景，并要求绘图与评测命令显式传入目标和对应场景参数。报告也分别给出中心距离和扣除机体半径后的净空。

**后续改进：** 评测脚本应直接读取 YAML 和场景元数据，减少硬编码。

---

## 7. 验证方法

## 7.1 编译验证

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

修改 controller 或 planner 后，不只检查命令返回值，还检查构建日志是否真正重新生成对应 `.cpp.o`。

必要时执行干净重建：

```bash
rm -rf build install log
colcon build --symlink-install
```

或对单个包执行目标级清理和重建。

## 7.2 动力学验证

动力学主要通过源码交叉检查、单元测试和实跑数据验证：

- 检查 RPM 到角速度转换；
- 检查单电机推力和反扭矩公式；
- 对照动力学和 mixer 的分配矩阵；
- 检查四个相同 RPM 时 roll、pitch、yaw 力矩应接近零；
- 检查悬停 RPM 是否与 `mg` 平衡量级一致；
- 检查电机一阶响应和 RPM 限幅；
- 检查姿态积分后四元数是否归一化；
- 检查状态中是否出现 NaN 或 Inf。

实跑中同时记录真值位置、姿态和实际 RPM，以确认动力学方向和电机编号没有反接。

## 7.3 控制器验证

控制器通过以下方式检查：

- 悬停实验：验证高度收敛和稳态误差；
- 单目标实验：验证平移方向、超调和姿态稳定性；
- 多目标实验：验证参考切换；
- RPM 曲线：检查 mixer 输出、饱和和异常归零；
- 暂停 planner：验证 reference timeout 的悬停保护；
- 检查 odom 超时和 reference 超时使用不同日志与处理分支；
- 对照控制公式和实际电机顺序检查 roll、pitch、yaw 符号。

## 7.4 ROS2 接口验证

```bash
ros2 topic list
ros2 topic info /drone/reference --verbose
ros2 topic info /drone/odom --verbose
ros2 topic echo --once /drone/odom
ros2 topic echo --once /drone/motor_rpm_cmd
```

重点检查：
- 节点是否全部启动；
- Topic 名称和消息类型；
- `/drone/motor_rpm_cmd` 是否包含四个值；
- `header.frame_id` 是否正确；
- 发布和订阅 QoS 是否兼容；
- 控制器和规划器是否使用 `/drone/odom`；
- 真值里程计是否只进入传感器仿真和评测；
- 静态地图是否可被后启动节点接收。

## 7.5 单目标回归

```bash
ros2 launch drone_visualization open.launch.py rviz:=false
```

在另一终端运行：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 2 1 1.5 \
  --duration 16 \
  --output /tmp/point.csv
```

独立审计阶段的一次结果为：

- 约 `3.19 s` 首次进入 `0.3 m`；
- 最终误差约 `0.046 m`；
- 无 RPM 饱和。

最低验收完整回归中的一次结果为：

- 约 `3.36 s` 首次进入 `0.3 m`；
- 最终误差约 `0.041 m`；
- 无 RPM 饱和。

不同运行之间因记录起点和噪声采样窗口不同可能存在小幅差异，因此报告采用对应提交结果文件中的同一次 CSV 与 metrics。

## 7.6 五障碍回归

```bash
ros2 launch drone_visualization sim.launch.py rviz:=false
```

在另一终端运行：

```bash
ros2 run drone_visualization run_experiment.py \
  --goal 8 0 1.5 \
  --duration 45 \
  --output /tmp/avoid.csv
```

修复过程中的一次验证结果包括：

- A* 与安全检查耗时约 `5.30 s`；
- 规划等待阶段高度约 `1.484–1.523 m`；
- 规划阶段平均 RPM 最低约 `5063`；
- 最终误差约 `0.037 m`；
- 中心最小障碍距离约 `0.776 m`；
- RPM 饱和样本为 `0/9001`；
- 不再发生零 RPM 坠地。

最低验收完整回归对应提交结果为：

- 最终误差约 `0.063 m`；
- 中心轨迹到原始 AABB 的最小距离约 `0.743 m`；
- 扣除 `0.18 m` 机体半径后净空约 `0.563 m > 0.5 m`；
- 无 RPM 饱和。


## 8. AI 未替代完成的工作

AI 没有替代作者完成以下工作：

- 决定项目主题、功能范围和总体架构；
- 决定主要算法及安全策略；
- 制定测试场景、评价指标和通过标准；
- 对开源控制器参考关系进行理解、适配和说明；
- 实际观察 RViz 中的飞行与避障行为；
- 确认电机编号、坐标系、控制符号和安全距离符合任务定义；
- 决定控制器和规划器参数是否适合最终演示；
- 对最终源码、运行二进制和实验数值进行人工复核；
- Git 提交、远端推送、许可证选择；
- PDF 排版和演示视频录制。

---

## 9. 总结

本项目中，AI 的主要价值在于提高代码审计、问题定位、实验执行和文档整理效率。五障碍坠地案例说明，AI 可以协助从日志和数据中发现跨节点实时性问题，也可以协助实现异步规划和安全降级。

同时，该案例也暴露了 AI 辅助开发的风险：源码看起来正确、AI 声称修改完成或历史构建曾经通过，都不能证明当前运行系统正确。对于 ROS2/C++ 项目，必须进一步确认：

1. 实际启动的软件包路径；
2. 可执行文件是否由当前源码重新构建；
3. 节点日志和 Topic 行为是否符合设计；
4. 完整时间序列中是否存在坠地、穿障、RPM 归零、饱和或姿态发散；
5. 正常场景和故障注入是否都能通过。

因此，本项目将 AI 输出视为待验证的开发建议，而不是最终结论。最终提交内容由作者依据源码、构建日志、ROS2 实跑、CSV、曲线、GTest 和实际演示结果确认。