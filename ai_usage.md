# AI 辅助编程使用说明

## 1. 声明

本仓库在开发、审计、问题定位、代码修改、测试和文档编写过程中使用了 AI 辅助工具。当前能够确认的主要工具是：

- **OpenAI Codex**：用于阅读 ROS2/C++/Python 代码、对照任务要求审计仓库、提出修改方案、编辑代码、执行构建与测试、分析 CSV/日志、编写 README 和实验报告。

AI 的输出没有被视为天然正确。涉及动力学公式、电机顺序、控制符号、ROS2 topic、规划安全逻辑和实验指标的内容，均通过源码交叉检查、编译、GTest、ROS2 实跑日志、二进制检查或 CSV 曲线验证。若作者在本次记录之外还使用过其他 AI 工具，应在正式提交前继续补充，不能用本文件隐瞒未记录的使用情况。

## 2. 关键 Prompt / 交互摘要

以下不是逐字聊天记录，而是对关键意图、AI 输出和实际验证的如实摘要。

| 编号 | Prompt 或交互摘要 | AI 主要工作 | 人工/工具验证 |
|---:|---|---|---|
| 1 | “根据完整任务书检查当前仓库是否满足任务目标。” | 扫描六个 ROS2 包、launch、配置、README、报告、测试和结果图片；逐项给出满足/部分满足/缺失结论 | `rg --files`、源码阅读、Git 状态、远端仓库检查 |
| 2 | “不要只看文档，验证是否真的能编译和测试。” | 执行 `colcon build`、`colcon test` 和 `colcon test-result`，核对动力学和规划测试 | 6 个包构建成功；汇总 `7 tests, 0 failures` |
| 3 | “验证普通目标点 `(2,1,1.5)` 是否满足 0.3 m。” | 无界面启动完整系统，记录真值位置和 RPM，计算首次进入阈值及最终误差 | 约 `3.19 s` 首次进入 `0.3 m`；最终误差约 `0.046 m`；无 RPM 饱和 |
| 4 | “检查五障碍 `(8,0,1.5)` 的安全距离和稳定性。” | 执行 42 秒五障碍实验，计算最终误差、路径长度、AABB 净空和 RPM；结合节点日志分析异常 | 发现参考超时后零 RPM、无人机从约 `1.52 m` 掉到 `0.05 m`，而非只依据最终到达结果判定成功 |
| 5 | “告诉我如何自己执行五障碍实验。” | 给出分终端启动、记录、绘图、误差计算、最低高度和低 RPM 检查命令 | 作者按命令启动 ROS2 系统并反馈实际 launch 日志 |
| 6 | “修复同步 A* 超过 2.5 秒导致零 RPM 坠地的问题。” | 将 A*、B-spline 安全检查和弧长表计算移入 `std::async` 后台任务；增加规划 generation；规划期间持续发布 `PLANNING_HOLD` | C++ 编译通过；规划测试通过；五障碍 A* 耗时约 `5.30 s` 时高度保持 |
| 7 | “参考超时也不能在空中直接关电机。” | 将 odom 超时和 reference 超时分开：odom 仍新鲜时锁定当前位置、速度/加速度置零；仅 odom 失效或无参考时输出零 RPM | 人工暂停规划器超过 `2.5 s`，控制器打印 `holding current position`，高度保持 `1.482–1.526 m` |
| 8 | “修复后仍出现旧警告并掉地。” | 对比用户日志与新源码预期日志，判断实际运行旧二进制；检查 ROS package prefix、软链接、二进制字符串和时间戳 | `strings` 发现二进制仍含旧文本；安装程序时间早于源码 |
| 9 | “确保修复源码真正被编译。” | 先尝试 `--cmake-clean-cache`，发现 controller 对象因旧 build 时间戳仍未重编译；随后执行 controller target clean 并重建 | 新二进制包含 `stale reference: holding current position`，planner 包含 `PLANNING_HOLD` |
| 10 | “这些加分项我已经做了哪些？” | 对照代码判断参数化、传感器噪声、voxel/LiDAR、测试已完成；外力和 waypoint 部分完成；多机、地面站、MARSIM 对比未完成 | 逐项链接到 `sim.yaml`、动力学、地图、LiDAR、测试和 waypoint 脚本 |
| 11 | “按照 6–10 页要求写详细 Markdown 报告。” | 整理系统架构图、topic 数据流、动力学/控制公式、地图规划、实验结果、失败案例、参考关系和反思 | 对照任务报告九项要求逐项检查；图片链接使用仓库现有结果 |
| 12 | “编写 `ai_usage.md`，说明 AI 做了什么和如何验证。” | 汇总本次 AI 使用、错误、修正、人工确认公式和验证命令 | 本文件与实际对话、Git diff、构建日志和实验结果交叉核对 |
| 13 | “README 中的实验命令无法执行，重新检查是否满足最低验收场景。” | 复现安装入口和错误行为，区分 `open/sim/narrow` 场景，重写记录/绘图/waypoint 脚本的诊断与指标输出，并逐项实跑五类场景 | 脚本无 ROS 数据和任务超时时返回非零；悬停、单目标、多目标、五障碍和强制爬升均产生新曲线与 metrics 文件 |

## 3. AI 协助完成或修改的内容

### 3.1 仓库审计和需求映射

AI 将任务书拆分为动力学、控制器、地图、规划、可视化、验收实验、Git 交付、PDF/视频和 AI 说明等检查项，并检查：

- package 是否独立、依赖是否完整；
- launch 是否真正启动全部节点；
- topic 名称和消息类型是否与文档一致；
- 代码是否真正包含任务要求的公式，而不只是 README 声称；
- 实验图片是否有对应的脚本和可复现命令；
- Git 是否错误提交 `build/`、`install/`、`log/`；
- 报告、PDF、视频和 `ai_usage.md` 是否存在。

### 3.2 异步规划安全修复

早期规划器在 ROS2 定时器回调中同步执行 A*。AI 协助完成：

1. 定义 `AsyncPlanResult`，只保存搜索结果、路径模式、B-spline、弧长表和显示采样；
2. 启动规划前复制膨胀地图、起点、目标和 generation；
3. 用 `std::async(std::launch::async, ...)` 执行 A* 和曲线检查；
4. 主线程使用非阻塞 `future.wait_for(0)` 轮询结果；
5. 搜索期间固定 `hold_position` 并持续发布参考；
6. 新目标使 generation 增加，旧结果完成后被丢弃；
7. 捕获后台任务异常并转换为 `PLAN_EXCEPTION`，避免直接崩溃；
8. 规划期间禁止 LiDAR 使用旧 spline 触发无效重复重规划。

### 3.3 控制器超时降级修复

AI 协助将原有单一 `fresh` 判断拆为：

- `odom_fresh`；
- `reference_fresh`。

当 odom 新鲜而 reference 过期时，控制器保存当前状态位置作为 `failsafe_reference.position`，将速度、加速度和 yaw rate 置零，重置控制器的期望姿态差分状态，然后继续计算悬停 RPM。该策略避免把“上游规划暂时无参考”误处理为“飞行器必须立即断电”。

### 3.4 实验与指标分析

AI 使用已有 Python 记录脚本生成 CSV，并用只读 shell/awk 统计：

- 最终位置误差；
- 首次进入 `0.3 m` 的时间；
- 最低/最高高度；
- 四电机平均最低转速；
- RPM 饱和样本数量；
- 相邻轨迹点累计长度；
- 轨迹点到五个原始障碍 AABB 的最小距离。

AI 同时重新生成了修复后的五障碍位置、误差、RPM、轨迹和安全距离图片。

### 3.5 文档

AI 协助扩写：

- README 中异步规划和超时悬停说明；
- `report/REPORT.md` 的架构图、公式、参数表、实验结果和失败案例；
- 本 `ai_usage.md`。

## 4. 作者重点确认或修改的核心内容

以下内容不能仅凭 AI 文字接受，而是结合源码、运行现象或任务接口进行了确认：

### 4.1 动力学公式与约定

- 世界系采用 ENU，推力沿机体 `+z`；
- 四元数表示机体系到世界系旋转；
- RPM 必须先乘 `2π/60` 转为 `rad/s`，再进入平方推力模型；
- 四电机顺序为前左、后左、后右、前右；
- X 型布局的 roll/pitch 力臂为 `arm_length/sqrt(2)`；
- yaw 反扭矩符号为 `+ - + -`；
- 动力学和控制器的分配矩阵必须完全一致；
- 四元数每步积分后归一化。

### 4.2 控制器接口和安全行为

- 控制器输入不是直接订阅用户 goal，而是订阅规划器输出的 `TrajectorySetpoint`；
- 位置环包含位置、速度反馈和加速度前馈；
- 姿态误差使用 SO(3) 反对称矩阵的 vee 映射；
- mixer 输出的是四电机 RPM，而不是 PWM 或归一化油门；
- 里程计失效与规划参考失效必须采用不同的降级策略；
- 空中 reference timeout 不能直接输出零 RPM。

### 4.3 地图和安全距离

- 地图是三维 voxel，而不是 2D OccupancyGrid；
- 默认地图分辨率 `0.2 m`；
- 原始安全距离为 `0.5 m`，无人机半径为 `0.18 m`；
- A* 中心搜索层总膨胀约 `0.68 m`；
- 报告的中心轨迹—障碍距离与机体表面净空是两种不同口径。

### 4.4 ROS2 接口

- `/drone/motor_rpm_cmd` 长度必须为 4；
- `/drone/ground_truth/odom` 不能直接作为控制反馈；
- `/drone/odom` 来自 ESKF；
- 路径和地图使用 `map` frame；
- A*、B-spline/安全折线和滚动预测分别发布独立 `nav_msgs/Path`；
- 静态地图和规划路径使用 Transient Local QoS，传感器点云使用 SensorData QoS。

## 5. AI 产生过的问题及修正

### 5.1 只验证源码，没有立即验证正在运行的二进制

在修复异步规划和控制器超时后，AI 已经成功编译并完成过一次回归。之后为了清理 Git 中被构建修改的生成文件，AI 将被版本控制的 `build/`、`install/`、`log/` 恢复到仓库版本。这使源码仍是修复版，但安装软链接重新指向旧二进制。

作者随后运行系统，日志仍出现：

```text
stale odometry/reference: commanding zero RPM
```

AI 起初不能仅凭“源码已修改”判断问题，最终通过以下证据发现错误：

```bash
ros2 pkg prefix drone_controller
readlink -f install/drone_controller/lib/drone_controller/position_controller_node
strings install/drone_controller/lib/drone_controller/position_controller_node
stat src/... install/...
```

旧二进制只包含旧警告文本，且时间早于源码。修正方式是强制清理 controller build target 并重新编译，随后使用 `strings` 确认新文本真正进入二进制。

**经验：** 对 C++/ROS2 项目，AI 完成修改后必须验证“实际启动文件”，不能只验证源码和一次历史构建。

### 5.2 `--cmake-clean-cache` 不保证旧对象文件重编译

AI 第一次尝试使用：

```bash
colcon build --cmake-clean-cache
```

但仓库中被跟踪的旧对象文件时间戳导致 controller 显示 `Built target`，实际没有重新编译源文件。通过检查构建输出没有出现 `Building CXX object ... position_controller_node.cpp.o`，并再次检查二进制字符串，发现仍是旧程序。

最终使用：

```bash
cmake --build build/drone_controller --target clean
colcon build --symlink-install --packages-select drone_controller
```

构建日志明确出现源码重新编译，二进制也包含新警告。

**经验：** 仓库不应提交 build/install/log；干净源码构建比依赖时间戳更可靠。

### 5.3 早期设计把耗时规划放在定时器线程

早期版本虽然能够最终到达目标，但把同步 A* 放在主回调中，导致五障碍规划期间 reference 停止发布。若只查看最终误差，会错误判断实验成功。通过 CSV 的高度/RPM 时间序列和控制器日志才发现无人机中途坠地。

**修正：** 后台规划、主线程悬停参考、控制器二级失联悬停，并新增失败注入回归。

### 5.4 指标脚本和文档命令曾存在口径问题

- 绘图脚本障碍物盒体是默认场景硬编码，因此不能直接用于 random/narrow 场景的净空计算；
- 单目标绘图若忘记传 `--goal 2 1 1.5`，误差曲线会按默认悬停目标计算；
- “安全距离”必须说明是中心轨迹到障碍还是机体表面到障碍。

报告中已明确这些限制，后续应让评测脚本直接读取 YAML 和场景元数据。

## 6. 验证方法

### 6.1 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

修改 controller/planner 后，还检查构建输出是否真的重新编译对应 `.cpp.o`。

### 6.2 单元测试

```bash
colcon test
colcon test-result --verbose
```

最近一次汇总为：

```text
7 tests, 0 errors, 0 failures, 0 skipped
```

### 6.3 ROS2 接口检查

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /drone/reference --verbose
ros2 topic echo --once /drone/odom
```

检查节点、topic 类型、frame_id 和 QoS 是否与 README 一致。

### 6.4 单目标回归

```bash
ros2 launch drone_visualization open.launch.py rviz:=false
ros2 run drone_visualization run_experiment.py \
  --goal 2 1 1.5 --duration 16 --output /tmp/point.csv
```

独立审计结果：最终误差约 `0.046 m`，约 `3.19 s` 首次进入 `0.3 m`，无 RPM 饱和。

### 6.5 五障碍回归

```bash
ros2 launch drone_visualization sim.launch.py rviz:=false
ros2 run drone_visualization run_experiment.py \
  --goal 8 0 1.5 --duration 45 --output /tmp/avoid.csv
```

修复后结果：

- A* 搜索约 `5.30 s`；
- 规划等待高度 `1.484–1.523 m`；
- 规划阶段平均 RPM 最低约 `5063`；
- 最终误差约 `0.037 m`；
- 中心最小障碍距离约 `0.776 m`；
- RPM 饱和样本 `0/9001`；
- 不再掉到地面。

### 6.6 失败注入

在悬停状态暂停 planner 进程，使 `/drone/reference` 超过 `2.5 s` 不更新。检查控制器是否打印：

```text
stale reference: holding current position instead of zero RPM
```

该实验中高度保持在 `1.482–1.526 m`，平均 RPM 最低约 `5053`，验证二级保护有效。

### 6.7 二进制真实性检查

```bash
strings install/drone_planner/lib/drone_planner/ab_planner_node \
  | grep PLANNING_HOLD

strings install/drone_controller/lib/drone_controller/position_controller_node \
  | grep "holding current position"
```

该检查用于防止 ROS2 启动旧 overlay 或旧构建产物。

### 6.8 最低验收场景完整回归

2026-07-18 对 README 命令做了从安装入口到绘图输出的完整复验。发现旧说明没有明确指出 launch 与记录器必须分两个终端，并错误地让单目标和 waypoint 容易沿用五障碍场景；旧绘图命令也没有传入单目标坐标，误差曲线会按默认悬停点计算。修正后结果为：

- 从地面悬停：最终误差约 `0.043 m`，最后 2 秒平均误差约 `0.034 m`；
- 单目标：最终误差约 `0.041 m`，约 `3.36 s` 首次进入 `0.3 m`；
- 多目标：起飞点加四个正方形航点 `5/5` 到达，总用时约 `16.44 s`；
- 五障碍：最终误差约 `0.063 m`，中心净空约 `0.743 m`，扣除机体半径后约 `0.563 m > 0.5 m`；
- 强制爬升：墙体附近高度超过 `4.0 m`，最终误差约 `0.039 m`，扣除机体半径后净空约 `0.758 m > 0.5 m`；
- 所有上述回归的 RPM 饱和样本均为 `0`。

最新结果保存在 `report/results/*_metrics.txt`，对应曲线由同一次 CSV 记录生成。

## 7. AI 未代替完成的工作

AI 没有替代以下责任：

- 作者对最终提交内容真实性的确认；
- 作者实际观察 RViz 飞行行为；
- 作者判断电机编号、坐标系和安全距离是否符合任务定义；
- 作者决定控制和规划参数是否适用于最终演示；
- PDF 排版和 1–3 分钟视频录制；
- Git 提交、远端推送和开源许可证选择；
- 对最终报告数值、PDF 排版和演示视频画面做提交前人工复核。

## 8. 总结

AI 对本项目最有价值的部分不是生成大量样板代码，而是把任务要求转化为可检查项，并通过构建、日志、CSV 和失败注入发现跨节点安全问题。与此同时，本项目也暴露了 AI 辅助开发的典型风险：源码修改正确不代表运行的二进制正确，最终结果必须由可复现命令和实际数据验证。
