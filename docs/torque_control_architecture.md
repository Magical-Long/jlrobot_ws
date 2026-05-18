# 力矩控制架构设计与当前实现

本文档描述当前工作空间在扩展到 **Gazebo 力矩控制** 时的代码组织方式、模块职责划分、当前已经落地的最小 PD 闭环实现，以及后续扩展方向。

当前实现的目标不是立即覆盖全部高级控制，而是先建立一套：

- 分层清晰
- 便于调试
- 方便从简单 PD 控制逐步升级到逆动力学 / 阻抗控制

的架构基础。

---

## 1. 设计目标与当前进度

当前项目已经具备：

- `robot_description`
  - Gazebo 仿真描述
  - `ros2_control` 接口
  - controller YAML
  - launch 文件

- `robot_core`
  - 运动学模型
  - IK
  - 轨迹插值
  - 关节空间 PD 力矩控制核心类

- `robot_ros`
  - 将算法层结果接到 ROS2 / Gazebo 的桥接节点
  - Gazebo effort 模式下的 PD 闭环控制节点

在此基础上，力矩控制扩展的核心目标是：

1. 在 **算法层** 独立实现控制律
2. 在 **ROS 层** 只做状态订阅、参数读取和命令发布
3. 在 **描述层** 只负责 effort 控制器与 Gazebo 启动接线

也就是说，要避免把“控制算法、Gazebo 接口、URDF 启动逻辑”全部混到一个节点里。

---

当前已经打通的最小链路是：

```text
/joint_states
    -> robot_ros::GazeboEffortControllerNode
    -> robot_core::ControlState
    -> robot_core::JointSpacePDController
    -> /arm_effort_controller/commands
    -> Gazebo + ros2_control
    -> /joint_states
```

也就是说，目前已经具备一个最小可运行的“关节空间 PD 力矩闭环”。

在此基础上，当前还额外打通了上一层“任务空间到关节参考”的桥接链路：

```text
/target_pose
    -> gazebo_pose_reference_controller
    -> IKSolver
    -> JointTrajectoryPlanner
    -> /desired_joint_states
    -> gazebo_effort_controller
    -> /arm_effort_controller/commands
```

这样系统就形成了一个清晰的两层结构：

- 上层负责把末端位姿目标转成关节参考轨迹
- 下层负责把关节参考轨迹转成实际力矩命令

---

## 2. 总体分层建议

推荐分成 4 层：

```text
robot_description   -> 模型描述与 Gazebo/ros2_control 接线
robot_core          -> 运动学 / 动力学 / 控制算法
robot_ros           -> ROS2 节点封装与 Gazebo 命令桥接
docs                -> 架构、接口、参数与调试文档
```

更具体一点：

```text
参考轨迹/目标
    -> 控制算法（robot_core）
    -> ROS2桥接节点（robot_ros）
    -> /arm_effort_controller/commands
    -> Gazebo + ros2_control
    -> /joint_states 反馈
    -> 控制算法闭环更新
```

---

## 3. 各功能包职责

### 3.1 `robot_description`

职责：

- 存放 URDF / Gazebo URDF
- 存放位置控制与力矩控制的 controller YAML
- 存放 Gazebo 启动 launch
- 定义机器人在 Gazebo 中使用的位置 / 力矩控制接口

不建议放入：

- 力矩控制数学公式
- PD / 逆动力学 / 阻抗控制逻辑
- 轨迹跟踪状态机

当前已经有：

- `robot_gazebo.urdf`
- `robot_gazebo_effort.urdf`
- `iiwa7_controllers.yaml`
- `iiwa7_effort_controllers.yaml`
- `robot_gazebo.launch.py`
- `robot_gazebo_effort.launch.py`

也就是说，这一层已经可以承担：

- **位置控制模式 Gazebo 启动**
- **力矩控制模式 Gazebo 启动**

### 3.2 `robot_core`

职责：

- 提供与 ROS2 无关的核心算法
- 提供控制器数学逻辑
- 提供机器人模型、运动学、动力学、控制状态结构

建议继续保留当前定位：

- `robot_model`
- `ik_solver`
- `trajectory_planner`

并新增：

- `dynamics_model`
- `torque_controller_base`
- `joint_space_pd_controller`
- `computed_torque_controller`
- `impedance_controller`
- `control_state`
- `control_reference`

也就是说，`robot_core` 将从“运动学核心包”扩展为“控制算法核心包”。

### 3.3 `robot_ros`

职责：

- 订阅 Gazebo / ROS2 状态
- 读取目标
- 调用 `robot_core` 控制算法
- 把输出力矩命令发布给 effort controller

不建议在这一层做：

- 大量动力学公式实现
- 控制器核心数学
- 复杂的模型推导

ROS 节点应更像“桥接器”和“运行时编排器”。

### 3.4 `docs`

职责：

- 保存系统架构文档
- 记录话题接口
- 记录参数意义
- 记录位置 / 速度 / 力矩控制模式差别
- 记录调试流程

---

## 4. 推荐新增模块设计

`robot_core/include/robot_core/` 当前已经落地或建议逐步形成如下结构：

```text
control_state.hpp
control_reference.hpp
torque_controller_base.hpp
joint_space_pd_controller.hpp
```

### 4.1 `control_state.hpp`

当前已实现一个统一控制状态结构：

```text
q
dq
```

用途：

- 表示当前关节位置和速度
- 供所有控制器统一读取

### 4.2 `control_reference.hpp`

当前已实现参考输入结构：

```text
q_des
dq_des
ddq_des
```

用途：

- 表示期望关节位置、速度、加速度
- 供不同控制器统一消费

### 4.3 `torque_controller_base.hpp`

当前已实现一个统一抽象接口：

```text
computeTorque(state, reference) -> tau
```

这样后续：

- PD 控制器
- 逆动力学控制器
- 阻抗控制器

都可以通过统一接口被 ROS2 节点调用。

### 4.4 `joint_space_pd_controller.hpp`

这一层已经实现，是当前最小闭环的核心控制律。

控制律可以从最基础版本开始：

```text
tau = Kp * (q_des - q) + Kd * (dq_des - dq)
```

优点：

- 简单
- 稳定
- 容易验证
- 能快速跑通 effort 模式 Gazebo 闭环

### 4.5 `computed_torque_controller.hpp`

作为第二阶段扩展：

```text
tau = M(q) * ddq_des + C(q, dq) + g(q) + feedback
```

前提：

- 已有可靠的动力学接口
- 已完成 PD 控制链基本验证

### 4.6 `impedance_controller.hpp`

第三阶段可扩展：

- 关节空间阻抗
- 笛卡尔空间阻抗

适合：

- 柔顺控制
- 接触任务
- 更高级控制实验

---

## 5. `robot_ros` 当前节点设计

当前已经新增：

- `gazebo_effort_controller`
- `gazebo_pose_reference_controller`
- `gazebo_effort_controller.launch.py`
- `gazebo_pose_reference_controller.launch.py`
- `gazebo_effort_control_demo.launch.py`
- `gazebo_pose_effort_control_demo.launch.py`

其中 `gazebo_effort_controller` 节点职责非常单一：

1. 订阅 Gazebo 的 `/joint_states`
2. 订阅目标参考 `/desired_joint_states`
3. 把 ROS 消息整理成 `ControlState` / `ControlReference`
4. 调用 `JointSpacePDController`
5. 发布力矩命令到 `/arm_effort_controller/commands`

这样 ROS2 层和控制算法层就被清楚分开了。

`gazebo_pose_reference_controller` 节点则位于它的上游，负责：

1. 订阅 `/target_pose`
2. 读取当前 `/joint_states`
3. 调用 IK 求目标关节角
4. 调用轨迹插值器生成一串关节参考点
5. 按固定采样周期发布到 `/desired_joint_states`

---

## 6. 当前控制链路说明

### 6.1 Gazebo 启动链

- `robot_description/launch/robot_gazebo_effort.launch.py`
  - 启动 Gazebo
  - 加载 `robot_gazebo_effort.urdf`
  - 启动 `joint_state_broadcaster`
  - 启动 `arm_effort_controller`

这一层只负责：
- effort 接口接线
- controller_manager 加载控制器
- Gazebo 物理仿真

### 6.2 ROS2 控制节点链

- `robot_ros/launch/gazebo_effort_controller.launch.py`
  - 启动 `gazebo_effort_controller`
  - 加载 `config/gazebo_effort_controller.yaml`

这一层只负责：
- 读取控制参数
- 接 ROS2 话题
- 调用 `robot_core` 算法

### 6.3 `gazebo_effort_controller` 节点内部流程

节点启动后，按以下顺序运行：

1. `declareParameters()`
   - 读取 `urdf_path`
   - 读取 `end_effector_frame`
   - 读取控制频率、话题名等

2. `initializeController()`
   - 加载 `RobotModel`
   - 创建 `JointSpacePDController`
   - 初始化 `current_state_`
   - 初始化 `reference_`

3. 订阅 `/joint_states`
   - 将 Gazebo 反馈的关节位置、速度整理成 `ControlState`

4. 若尚未收到外部目标且 `hold_current_position_on_startup=true`
   - 自动把当前姿态复制成目标姿态
   - 这样 effort 模式启动后不会立刻因零力矩而自然下垂

5. 订阅 `/desired_joint_states`
   - 将外部给出的目标关节角、目标关节速度整理成 `ControlReference`

6. 定时器周期执行 `runControlLoop()`
   - 调 `JointSpacePDController::computeTorque(...)`
   - 得到每个关节的 `tau`
   - 发布到 `/arm_effort_controller/commands`

### 6.4 PD 控制律

当前实现的控制律是：

```text
tau = Kp * (q_des - q) + Kd * (dq_des - dq)
```

其中：

- `q`：当前关节角
- `dq`：当前关节速度
- `q_des`：目标关节角
- `dq_des`：目标关节速度
- `tau`：输出关节力矩

工程上又补了一个力矩限幅：

```text
tau_i = clamp(tau_i, -max_torque_i, max_torque_i)
```

这样初期调试更安全，不容易一上来就力矩过猛。

---

## 7. 参数文件

当前 PD 力矩控制器的默认参数文件为：

- `robot_ros/config/gazebo_effort_controller.yaml`

其中主要参数包括：

- `joint_state_topic`
- `reference_topic`
- `command_topic`
- `control_rate_hz`
- `hold_current_position_on_startup`
- `kp`
- `kd`
- `enable_torque_limits`
- `max_torque`

这样做的目的，是把“调参”从 launch 和 C++ 里拿出来，方便后续反复试验。

---

## 8. 推荐启动方式

一键启动 Gazebo effort 模式和 PD 控制节点：

```bash
ros2 launch robot_ros gazebo_effort_control_demo.launch.py
```

单独启动 Gazebo effort 模式：

```bash
ros2 launch robot_description robot_gazebo_effort.launch.py
```

单独启动 PD 力矩控制节点：

```bash
ros2 launch robot_ros gazebo_effort_controller.launch.py
```

---

## 9. 目标参考输入方式

当前最小闭环版本约定上层参考输入为：

- 话题：`/desired_joint_states`
- 类型：`sensor_msgs/msg/JointState`

至少应提供：

- `name`
- `position`

可选提供：

- `velocity`

这意味着上层规划器、轨迹生成器、示教节点，后续都可以统一向这个话题发目标关节参考。

---

## 10. 后续扩展顺序建议

建议继续按下面顺序扩展，而不是一步跳到复杂控制：

1. 先稳定调通 `JointSpacePDController`
2. 再加入重力补偿
3. 再做 `computed_torque_controller`
4. 再做阻抗控制

这样每一步都能清楚区分：

- 是 Gazebo 接线问题
- 还是 ROS2 桥接问题
- 还是控制律本身的问题

建议新增一个独立节点，而不是把力矩控制逻辑塞进现有 `gazebo_pose_controller`。

推荐名字：

```text
gazebo_effort_controller_node
```

职责：

1. 订阅 `/joint_states`
2. 更新当前状态 `q / dq`
3. 获取期望参考轨迹 `q_des / dq_des / ddq_des`
4. 调用 `robot_core` 控制器计算 `tau`
5. 发布到：

```text
/arm_effort_controller/commands
```

这样位置控制节点和力矩控制节点在职责上完全分离：

- `gazebo_pose_controller`
  - 面向位置轨迹控制器

- `gazebo_effort_controller_node`
  - 面向 effort 控制器

---

## 6. 推荐节点运行流程

力矩控制节点建议按如下流程运行：

### 6.1 初始化阶段

1. 读取参数
2. 加载 `RobotModel`
3. 初始化 `JointSpacePDController` 或其他控制器
4. 创建：
   - `/joint_states` 订阅器
   - 目标参考输入接口
   - `/arm_effort_controller/commands` 发布器
5. 创建控制循环定时器

### 6.2 控制循环阶段

每个控制周期：

1. 读取最新 `q / dq`
2. 读取当前参考 `q_des / dq_des / ddq_des`
3. 调用控制器：

```text
tau = controller.computeTorque(state, reference)
```

4. 发布 `tau`
5. 进入下一周期

这是一条真正的闭环控制链，而不是“解一次 IK -> 发一条轨迹 -> 等待执行结束”的开环式轨迹执行。

---

## 7. 推荐消息与接口设计

### 7.1 最小版本

最小 effort 控制可以先不引入复杂 action / service，只用：

- 输入：
  - 参数配置
  - 或内部固定参考
- 状态输入：
  - `/joint_states`
- 输出：
  - `/arm_effort_controller/commands`

输出消息通常为：

```text
std_msgs/msg/Float64MultiArray
```

每个元素对应一个关节的力矩命令。

### 7.2 后续可扩展

如果以后需要更强的外部接口，再考虑增加：

- 目标关节姿态话题
- 目标末端位姿话题
- service 调用接口
- action 接口

但第一阶段建议先让控制链简单、透明。

---

## 8. 为什么不建议直接复用 `gazebo_pose_controller`

因为它当前的定位是：

- 接收目标末端位姿
- IK 求目标关节角
- 生成关节轨迹
- 发给位置控制器

这条链和力矩控制的时序与责任不同：

### 位置控制链

```text
目标位姿 -> IK -> 轨迹 -> 位置控制器执行
```

### 力矩控制链

```text
目标参考 -> 闭环计算 tau -> effort 控制器执行
```

如果硬把二者塞在同一个节点里，后面会变得非常混乱：

- 一部分逻辑是轨迹规划
- 一部分逻辑是实时闭环控制
- 一部分逻辑是 IK
- 一部分逻辑是 torque 计算

可维护性会迅速下降。

因此，建议：

- **位置控制节点保留**
- **力矩控制节点独立新增**

---

## 9. 推荐开发顺序

建议按以下顺序推进：

### 阶段 1：Joint-space PD 力矩控制

目标：

- 能在 Gazebo effort 模式下稳住一个目标关节构型

实现内容：

- `ControlState`
- `ControlReference`
- `TorqueControllerBase`
- `JointSpacePDController`
- `gazebo_effort_controller_node`

这是最推荐的第一步。

### 阶段 2：轨迹参考输入

目标：

- 让力矩控制不只“定点站住”，还能跟踪关节参考轨迹

实现内容：

- 轨迹参考生成器
- `q_des / dq_des / ddq_des` 随时间变化

### 阶段 3：动力学补偿

目标：

- 提升跟踪性能

实现内容：

- `DynamicsModel`
- 重力补偿
- 逆动力学前馈

### 阶段 4：高级控制

目标：

- 更真实、更先进的控制能力

实现内容：

- computed torque
- impedance control
- cartesian control

---

## 10. 推荐目录草图

推荐未来演化到如下结构：

```text
src/robot_core/include/robot_core/
  robot_model.hpp
  ik_solver.hpp
  trajectory_planner.hpp
  control_state.hpp
  control_reference.hpp
  torque_controller_base.hpp
  joint_space_pd_controller.hpp
  computed_torque_controller.hpp
  impedance_controller.hpp
  dynamics_model.hpp

src/robot_core/src/
  robot_model.cc
  ik_solver.cc
  trajectory_planner.cc
  joint_space_pd_controller.cc
  computed_torque_controller.cc
  impedance_controller.cc
  dynamics_model.cc

src/robot_ros/src/
  gazebo_pose_controller.cc
  gazebo_effort_controller.cc
```

---

## 11. 总结

对你当前项目来说，最合理的架构原则是：

1. `robot_description`
   - 只管描述与仿真接线

2. `robot_core`
   - 只管控制算法、模型与数学

3. `robot_ros`
   - 只管 ROS2 节点、话题和 Gazebo 桥接

4. 位置控制与力矩控制节点分离
   - 不要混成一个“大杂烩节点”

因此，下一步最值得实现的不是直接上复杂逆动力学，而是：

**先实现一个独立的 `JointSpacePDController + gazebo_effort_controller_node` 最小闭环版本。**

这会让：

- Gazebo effort 模式可运行
- 控制链清晰
- 后续加入重力补偿 / computed torque / impedance 时不需要推倒重来
