# 系统架构说明

本文档描述当前工作空间中 `robot_description`、`robot_core`、`robot_ros` 与 Gazebo / `ros2_control` 之间的关系，重点说明：

- 参数说明
- 控制流程
- 话题接口
- launch 启动流程
- Gazebo 交互流程

---

## 1. 总体架构

当前系统可以分成 4 层：

1. **模型描述层**
   - 包：`robot_description`
   - 作用：提供 URDF、Gazebo URDF、控制器 YAML、world 和 launch 文件。

2. **算法层**
   - 包：`robot_core`
   - 作用：提供 `RobotModel`、`IKSolver`、`JointTrajectoryPlanner` 等核心算法能力。

3. **ROS 桥接层**
   - 包：`robot_ros`
   - 作用：把上层目标位姿转换成 Gazebo 可执行的关节轨迹命令。

4. **仿真执行层**
   - 组件：Gazebo、`gazebo_ros2_control`、`controller_manager`、`arm_controller`、`joint_state_broadcaster`
   - 作用：执行关节轨迹、更新机器人状态、发布 `/joint_states`。

整体数据流如下：

```text
目标位姿 /target_pose
    -> robot_ros/gazebo_pose_controller
    -> robot_core::IKSolver
    -> robot_core::JointTrajectoryPlanner
    -> /arm_controller/joint_trajectory
    -> ros2_control / Gazebo
    -> /joint_states
    -> robot_state_publisher / TF / Gazebo状态更新
```

---

## 2. 参数说明

### 2.1 `robot_description` 包参数

主要出现在 `robot_gazebo.launch.py` 中。

- `use_sim_time`
  - 是否使用 Gazebo 仿真时钟。

- `gui`
  - 是否启动 Gazebo 图形界面。

- `pause`
  - Gazebo 是否以暂停状态启动。

- `world`
  - Gazebo 世界文件路径，默认使用 `custom_world.world`。

- `entity_name`
  - 机器人在 Gazebo 中生成时的实体名字。

- `robot_description_topic`
  - `spawn_entity.py` 读取机器人模型描述的 topic。

- `controller_manager_name`
  - controller spawner 连接的 `controller_manager` 名称。

- `x / y / z`
  - 机器人生成时在 Gazebo 世界坐标系下的初始位置。

### 2.2 `robot_ros/gazebo_pose_controller` 节点参数

主要出现在 `gazebo_pose_controller.launch.py` 和 `gazebo_pose_controller.cc` 中。

- `urdf_path`
  - `robot_core::RobotModel` 加载的 URDF 路径。
  - 默认使用 `robot_description/urdf/robot.urdf`。

- `end_effector_frame`
  - IK 使用的末端 frame 名称。
  - 当前默认：`iiwa_link_ee`

- `target_pose_topic`
  - 目标末端位姿输入话题。
  - 当前默认：`/target_pose`

- `joint_state_topic`
  - Gazebo 当前关节状态输入话题。
  - 当前默认：`/joint_states`

- `command_topic`
  - 发送给 `arm_controller` 的轨迹命令话题。
  - 当前默认：`/arm_controller/joint_trajectory`

- `segment_duration`
  - 从当前关节角到目标关节角这一段轨迹的总时长。

- `sample_period`
  - 轨迹离散采样周期。

- `position_only`
  - 是否只做末端位置控制。
  - `true` 时优先走 position-only IK。

- `ik_solver_method`
  - IK 数值方法。
  - 当前支持：
    - `dls`
    - `lm`

- `ik_max_iterations`
  - 单次 IK 最大迭代次数。

- `ik_position_tolerance`
  - 位置误差容差。

- `ik_orientation_tolerance`
  - 姿态误差容差。

- `ik_damping`
  - 数值 IK 阻尼系数。

- `ik_step_size`
  - 迭代步长。

- `enable_joint_limit_nullspace`
  - 是否启用关节限位回避零空间项。

- `joint_limit_nullspace_gain`
  - 零空间回避增益。

### 2.3 `robot_core::IKSolver` 核心参数

定义在 `robot_core/include/robot_core/ik_solver.hpp` 中的 `IKOptions`。

主要作用：

- 控制 IK 收敛速度
- 控制姿态 / 位置误差阈值
- 控制奇异位形附近的稳定性
- 控制是否偏向关节中间区域

---

## 3. 控制流程

### 3.1 节点初始化流程

入口文件：

- `robot_ros/src/gazebo_pose_controller_main.cc`

流程：

1. `rclcpp::init(...)`
2. 创建 `GazeboPoseControllerNode`
3. `rclcpp::spin(...)`

### 3.2 `GazeboPoseControllerNode` 初始化流程

主实现文件：

- `robot_ros/src/gazebo_pose_controller.cc`

构造函数中依次完成：

1. `declareParameters()`
   - 声明并读取 ROS 参数。

2. `initializeControllers()`
   - 创建 `RobotModel`
   - 加载 URDF
   - 创建 `IKSolver`
   - 创建 `JointTrajectoryPlanner`

3. 创建 `/joint_states` 订阅器
   - 回调：`handleJointState(...)`

4. 创建 `/target_pose` 订阅器
   - 回调：`handleTargetPose(...)`

5. 创建 `/arm_controller/joint_trajectory` 发布器

### 3.3 目标位姿控制流程

当节点收到一个目标位姿后，执行流程如下：

1. `handleTargetPose(...)`
   - 读取目标位姿
   - 选择当前起始关节角：
     - 若已经收到过 `/joint_states`，则使用 `current_configuration_`
     - 否则使用 neutral configuration

2. `solveTargetConfiguration(...)`
   - 准备多组 `seed`
   - 逐个尝试 IK
   - 若 `position_only=true`，优先走 position-only IK
   - 若求解成功，得到 `target_configuration`

3. `buildTrajectoryMessage(...)`
   - 将 `start_configuration` 与 `target_configuration` 交给 `JointTrajectoryPlanner`
   - 生成离散轨迹点
   - 转换成 `trajectory_msgs::msg::JointTrajectory`

4. 发布到 `/arm_controller/joint_trajectory`

5. Gazebo 中的控制器执行轨迹

### 3.4 关节状态更新流程

当 Gazebo 发布 `/joint_states` 后：

1. `handleJointState(...)` 被触发
2. `updateCurrentConfigurationFromJointState(...)`
   - 按关节名匹配顺序提取位置值
   - 更新 `current_configuration_`

这样下一次收到新目标位姿时，IK 就能从当前真实姿态开始求解。

---

## 4. 话题接口

### 4.1 输入话题

#### `/target_pose`

- 类型：`geometry_msgs/msg/Pose`
- 作用：外部向 `gazebo_pose_controller` 发送目标末端位姿

示例：

```yaml
position:
  x: 0.35
  y: 0.10
  z: 0.55
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

#### `/joint_states`

- 类型：`sensor_msgs/msg/JointState`
- 来源：`joint_state_broadcaster`
- 作用：
  - 给 `gazebo_pose_controller` 提供当前关节角
  - 给 `robot_state_publisher` 提供 TF 更新依据

### 4.2 输出话题

#### `/arm_controller/joint_trajectory`

- 类型：`trajectory_msgs/msg/JointTrajectory`
- 作用：将关节轨迹命令发送给 Gazebo 中的 `arm_controller`

这是当前位姿控制节点最核心的输出接口。

### 4.3 状态反馈话题

#### `/arm_controller/state`

- 类型：控制器内部状态消息
- 作用：反映轨迹控制器当前的期望值、实际值、误差等

#### `/joint_states`

- 类型：`sensor_msgs/msg/JointState`
- 作用：反馈机器人当前真实关节状态

#### `/tf` / `/tf_static`

- 来源：`robot_state_publisher`
- 作用：供 RViz、TF 树和其他依赖 TF 的节点使用

---

## 5. launch 启动流程

### 5.1 Gazebo 启动流程

启动文件：

- `robot_description/launch/robot_gazebo.launch.py`

流程如下：

1. 读取 `robot_gazebo.urdf`
2. 将控制器 YAML 路径注入 URDF 中的 Gazebo 插件参数
3. 启动 Gazebo 官方 `gazebo.launch.py`
4. 启动 `robot_state_publisher`
5. 用 `spawn_entity.py` 在 Gazebo 中生成机器人实体
6. 延时启动 `joint_state_broadcaster`
7. 再启动 `arm_controller`

### 5.2 位姿控制节点启动流程

启动文件：

- `robot_ros/launch/gazebo_pose_controller.launch.py`

流程如下：

1. 声明控制节点参数
2. 启动 `gazebo_pose_controller`
3. 节点开始等待：
   - `/joint_states`
   - `/target_pose`

### 5.3 推荐启动顺序

1. 启动 Gazebo 和机器人

```bash
ros2 launch robot_description robot_gazebo.launch.py
```

2. 启动位姿控制节点

```bash
ros2 launch robot_ros gazebo_pose_controller.launch.py
```

3. 发布目标位姿

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/Pose ...
```

---

## 6. Gazebo 交互流程

### 6.1 Gazebo 与 `ros2_control` 的连接

URDF 文件：

- `robot_description/urdf/robot_gazebo.urdf`

其中包含：

1. `ros2_control` 关节接口定义
   - 为每个关节声明：
     - `command_interface: position`
     - `state_interface: position`
     - `state_interface: velocity`

2. `gazebo_ros2_control` 插件
   - 负责把 Gazebo 里的关节与 ROS2 控制器系统接起来

### 6.2 Gazebo 执行控制命令的过程

当 `/arm_controller/joint_trajectory` 收到一条轨迹命令后：

1. `arm_controller` 解析轨迹
2. `ros2_control` 将每个轨迹点转换为关节位置命令
3. Gazebo 中的机械臂关节按命令运动
4. Gazebo 返回当前关节状态
5. `joint_state_broadcaster` 将状态发布到 `/joint_states`
6. `robot_state_publisher` 根据 `/joint_states` 更新 TF
7. Gazebo 界面和 ROS 侧状态一起反映机器人最新姿态

### 6.3 Gazebo 闭环效果

整条闭环链路如下：

```text
/target_pose
  -> gazebo_pose_controller
  -> IK / 轨迹规划
  -> /arm_controller/joint_trajectory
  -> arm_controller
  -> gazebo_ros2_control
  -> Gazebo 机械臂关节运动
  -> /joint_states
  -> gazebo_pose_controller 更新 current_configuration_
  -> robot_state_publisher 更新 TF
```

这意味着：

- 控制节点不是直接“操作模型显示”
- 控制节点只是发关节轨迹命令
- 真正执行运动的是 Gazebo + `ros2_control`
- 真正反馈状态的是 `/joint_states`

---

## 7. 当前系统的关键设计点

### 7.1 `current_configuration_` 的意义

`current_configuration_` 是 `gazebo_pose_controller` 中保存的“当前机器人真实关节角”。

它的作用：

1. 作为 IK 求解的优先 seed
2. 作为轨迹规划的起点
3. 保证连续多次发送目标时，轨迹从当前姿态继续运动，而不是每次都从 neutral 开始

### 7.2 `seed` 的意义

`seed` 是数值 IK 的初始关节角猜测。

当前系统中常见 seed 包括：

1. 当前关节状态
2. neutral configuration
3. 几组已知可达的示例关节姿态

多 seed 的意义在于：

- 同一个目标位姿，不同初值可能收敛到不同解
- 有些目标从当前姿态难收敛
- 但从别的已知可达姿态可能更容易收敛

### 7.3 `position_only` 的意义

`position_only=true` 表示控制重点是末端位置，而不是末端姿态。

这在 Gazebo 简单位置控制里很常见，因为：

- 对很多测试任务来说，只要求末端到达某个点
- 不需要同时满足严格姿态约束
- 这样通常能提升可达性和数值稳定性

---

## 8. 总结

当前系统的本质是一个“位姿到轨迹”的桥接架构：

1. `robot_description`
   - 定义机器人模型、控制器和 Gazebo 仿真环境

2. `robot_core`
   - 提供运动学、IK 和轨迹插值能力

3. `robot_ros`
   - 把目标末端位姿转成 `arm_controller` 可执行的轨迹命令

4. Gazebo + `ros2_control`
   - 真正执行轨迹并反馈机器人状态

因此，整个系统既不是单纯的“数学求解器”，也不是单纯的“Gazebo 显示器”，而是：

**目标位姿输入 -> 数值 IK -> 轨迹生成 -> Gazebo 执行 -> 状态反馈** 的完整闭环控制链。
