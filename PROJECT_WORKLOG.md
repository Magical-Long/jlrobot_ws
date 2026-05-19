# 项目工作日志

## 日期
2026-05-17

## 今日主题
Gazebo effort 控制链路调试、重力补偿接入、速度滤波实验与可视化调试

## 今日完成

### 1. 排查并修复位姿参考控制链路中的问题
- 排查了 `pose_effort_controller` 启动时参数 YAML 未正确安装的问题。
- 修复了 `gazebo_pose_reference_controller` 中“IK 已解出但参考轨迹构建失败”的问题。
- 原因是 `joint_states` 还原出的当前关节配置可能轻微越界，轨迹规划器严格检查限位后直接拒绝。
- 处理方式：
  - 在读取当前关节状态后先做 `clampToLimits()`
  - 在生成参考轨迹前再次对 `start/goal` 进行限幅
  - 增强了失败日志，便于定位后续维度/限位问题

### 2. 修复 launch / install 资源重复问题
- 发现 `robot_ros` 的 launch 文件在 `install/share/robot_ros/` 根目录与 `launch/` 子目录中重复安装。
- 这会导致 `ros2 launch` 报同名文件 found more than once。
- 修复内容：
  - 调整 `CMakeLists.txt` 安装路径
  - 明确把 `launch` 装到 `share/robot_ros/launch`
  - 明确把 `config` 装到 `share/robot_ros/config`
  - 清理旧安装残留

### 3. 明确 effort 控制链路的分层关系
- 上层：
  - `gazebo_pose_reference_controller`
  - 功能是 `target_pose -> desired_joint_states`
- 下层：
  - `gazebo_effort_controller`
  - 功能是 `joint_states + desired_joint_states -> torque commands`
- 认识到只启动上层桥接节点并不会自动输出力矩命令，必须单独启动下层 effort controller。

### 4. 加入末端位姿 debug 话题
- 在 `gazebo_pose_reference_controller` 中增加了调试用 publisher。
- 作用：
  - 持续发布当前 `iiwa_link_ee` 的真实世界坐标位姿
  - 便于直接将 `/target_pose` 和真实末端位姿进行对比
- 话题：
  - `/debug/current_ee_pose`
- 这块被明确标注为 debug 逻辑，后续可以删除。

### 5. 明确当前默认只做位置 IK，不做姿态控制
- 通过 YAML 和代码确认：
  - `position_only: true`
- 这意味着：
  - 上层 IK 默认只约束末端位置
  - 姿态保持当前姿态，不主动跟踪目标四元数
- 因此观察 `link7` 或 `iiwa_link_ee` 时，不能把位置到达误差和姿态约束问题混为一谈。

### 6. 分析纯关节 PD 力矩控制的局限性
- 当前 effort 控制器原始形式为：
  - `tau = Kp(q_des - q) + Kd(dq_des - dq)`
- 后来加入重力补偿后变为：
  - `tau = Kp(q_des - q) + Kd(dq_des - dq) + g(q)`
- 认识到纯 PD 即使不打满力矩，也可能稳定在错误平衡点。
- 原因可能包括：
  - 模型误差
  - 摩擦/阻尼
  - 无积分项导致稳态误差
  - 速度噪声造成阻尼项抖动

### 7. 接入 Pinocchio 重力补偿
- 在 `robot_core::JointSpacePDController::computeTorque()` 中接入 Pinocchio 动力学求解。
- 使用方式：
  - `pinocchio::rnea(model, data, q, 0, 0)`
- 得到当前构型下的重力补偿力矩 `g(q)`。
- 认识：
  - 该返回值是 7 维关节力矩向量
  - 不需要手工“分配到各关节”
  - 在理论上可以直接叠加到 PD 输出上

### 8. 重新核对力矩限幅配置
- 发现之前 `max_torque` 统一配置成 150 不合理。
- 根据 `robot_description/urdf/robot.urdf` 中每个关节的 `effort` limit，确认 7 个关节均为 300。
- 已将 YAML 中的 `max_torque` 调整为：
  - `[300, 300, 300, 300, 300, 300, 300]`

### 9. 在 robot_core 中建立通用滤波器模块
- 新增滤波器接口和实现文件：
  - `robot_core/include/robot_core/filters.hpp`
  - `robot_core/src/filters.cc`
- 当前支持：
  - 一阶低通滤波器 `LowPassFilter`
  - 一欧元滤波器 `OneEuroFilter`
  - 统一工厂接口 `createScalarFilter(...)`
- 设计思路：
  - 先抽象成标量滤波器
  - 每个关节各持有一个滤波器实例
  - 后续便于扩展更多滤波器

### 10. 在关节速度反馈入口加入滤波
- 在 `gazebo_effort_controller` 中对 `joint_states.velocity` 进行逐关节滤波。
- 滤波位置：
  - 进入 `current_state_.velocities` 之前
- 原因：
  - 降低速度噪声
  - 避免 `Kd * (dq_des - dq)` 直接放大高频毛刺

### 11. 为滤波效果增加 debug 话题
- 增加两路调试话题，方便用 PlotJuggler 对照：
  - `/debug/raw_joint_velocity`
  - `/debug/filtered_joint_velocity`
- 都使用 `sensor_msgs/JointState`
- 用于观察：
  - 原始速度噪声
  - 滤波后速度是否更平滑
  - 是否存在明显滞后

### 12. 对比低通滤波器与一欧元滤波器
- 尝试将默认滤波器从 `one_euro` 切换到 `low_pass`
- 通过 PlotJuggler 观察结果：
  - 一阶低通更接近原始波形，但噪声残留较多
  - 一欧元滤波器在当前场景下抑噪更强、曲线更平滑
- 当前判断：
  - 对本项目目前的速度反馈噪声情况，一欧元滤波器更合适

## 今日收获

### 1. 对控制器分层的理解更清晰
- 上层位姿控制器并不直接输出力矩
- 下层 effort controller 才真正决定关节力矩命令

### 2. 对重力补偿的理解更深入
- 重力项 `g(q)` 与当前构型 `q`、质量、惯量、质心有关
- 可以通过 Pinocchio 直接计算
- 本质上是“当前姿态下维持静止平衡所需的关节力矩”

### 3. 对逆动力学控制的认识更清楚
- 当前系统还只是 `PD + g(q)`，并不是完整 inverse dynamics / computed torque
- 完整逆动力学控制需要显式使用期望加速度 `ddq_des`
- 逆动力学不是看“关节角增量”，而是基于当前绝对状态 `q, dq` 求力矩

### 4. 对滤波器的选择有了更实际的判断标准
- 不是理论上更高级就一定更适合
- 应该以当前任务中的噪声形态、零点附近表现、时滞和稳定性为依据
- 在这个项目里，一欧元滤波器目前比固定低通更合适

## 当前遗留问题
- 虽然重力补偿和速度滤波已接入，但零点附近仍存在摇摆与抖动问题。
- 关节速度噪声虽有改善，但控制器在接近目标时仍可能偏离零位。
- 需要进一步区分：
  - `Kp/Kd` 参数问题
  - 模型误差问题
  - 速度噪声与阻尼项耦合问题
  - 是否需要积分项或更完整的逆动力学前馈
（- 5.18已经解决，引入逆动力学建模后不存在抖动和摇摆）

## 备注
### 1. 一欧元滤波器和低通滤波器的具体原理和频率与现实调试的联系掌握未牢固，需要补充一下

## 日期
2026-05-18

## 今日主题
补全主 URDF 动力学参数、统一日志接口，并梳理后续力控制方向

## 今日完成

### 1. 对齐 `robot.urdf` 与 `robot_gazebo_effort.urdf` 的动力学参数
- 重新核对了两份 URDF 的职责区别：
  - `robot.urdf` 作为 Pinocchio / IK / FK / 控制器内部模型使用
  - `robot_gazebo_effort.urdf` 作为 Gazebo effort 仿真包装模型使用
- 发现此前主 `robot.urdf` 中只有基座 `iiwa_link_0` 带有 `inertial`，活动连杆 `iiwa_link_1 ~ iiwa_link_7` 缺少完整质量和惯性参数。
- 将 `robot_gazebo_effort.urdf` 中机器人本体的惯性参数补回到 `robot.urdf`，包括：
  - 每个 link 的质心 `origin`
  - `mass`
  - `inertia`
- 同时为 `iiwa_link_ee` 补充了零质量虚拟 link 的惯性定义，用于与 Gazebo effort 模型接口保持一致。
- 保持 `robot.urdf` 仍然是“纯机器人模型”，没有把 `world`、`ros2_control`、Gazebo 插件等仿真包装层混入其中。

### 2. 重新验证 inverse dynamics 控制链路
- 在补齐主 URDF 动力学参数后，继续使用现有的：
  - `joint_effort_controller.launch.py`
  - `pose_effort_controller.launch.py`
- 因为这两套 launch 默认都读取 `robot.urdf`，所以现在 Pinocchio 终于能基于完整动力学模型进行 `rnea(...)` 逆动力学求解。
- 这一步没有改 Gazebo 模型结构，只是修正了控制器内部动力学模型不完整的问题。

### 3. 在 `robot_utils` 中建立通用增强日志模块
- 新建了底层通用日志器文件：
  - `robot_utils/include/robot_utils/logging.hpp`
  - `robot_utils/src/logging.cc`
- 日志器当前支持：
  - 日志等级：`Debug / Info / Warn / Error / Fatal`
  - 文件名、函数名、行号自动注入
  - 时间戳
  - 线程 ID
  - ANSI 颜色输出
  - 节流日志
- 通过宏提供统一调用方式，例如：
  - `ROBOT_UTILS_LOG_INFO_TAG(...)`
  - `ROBOT_UTILS_LOG_WARN_THROTTLE_MS_TAG(...)`

### 4. 为日志器加入外部配置能力
- 在 `robot_utils/config/` 中新增日志配置文件：
  - `robot_utils/config/logging.conf`
- 在 `Logger` 中补充了：
  - `loadConfigFromFile(...)`
  - `parseLogLevel(...)`
  - `parseBool(...)`
- 当前可以通过配置文件控制：
  - `min_level`
  - `enable_color`
  - `show_timestamp`
  - `show_thread_id`
  - `show_file_line`
  - `show_function`
  - `flush_every_line`
- 明确了一个重要点：
  - 如果没有显式调用 `loadConfigFromFile()`，当前日志系统仍使用 `LogConfig` 的默认值
  - 默认 `min_level = Info`，因此 `Debug` 日志默认不会输出

### 5. 将 `robot_ros` 的日志打印迁移到 `robot_utils`
- 修改 `robot_ros` 构建依赖，使其依赖 `robot_utils`
- 将以下节点中的 `RCLCPP_*` 日志迁移为统一日志宏：
  - `gazebo_effort_controller`
  - `gazebo_pose_effort_controller`
  - `gazebo_pose_controller`
  - `joint_state_demo`
- 保持了原有控制逻辑和流程判断不变，只替换了打印接口
- 给不同节点补充了模块标签，例如：
  - `EFFORT`
  - `POSE_EFFORT`
  - `POSE_TRAJ`
  - `JOINT_DEMO`

### 6. 优化 `gazebo_pose_effort_controller` 的日志行为
- 对重复 `/target_pose` 输入加入“重复目标抑制”，避免反复重建相同轨迹
- 将原先一条目标触发的多条 IK 日志收敛为一条成功摘要
- 保留失败日志和调试级 IK 细节日志，但默认不刷屏
- 补充了配置参数：
  - `repeated_target_position_tolerance`
  - `repeated_target_orientation_tolerance`

### 7. 梳理了 `robot_ros` 中各控制节点的职责
- 重新明确了两条不同控制链：
  - `gazebo_pose_controller`
    - 直接将位姿目标转成 `JointTrajectory`，发给 Gazebo 轨迹控制器
  - `gazebo_pose_effort_controller + gazebo_effort_controller`
    - 上层生成关节目标
    - 下层自行计算力矩命令
- 统一了相关命名和文案，强调 `pose_effort` 这条链是“上层位姿目标 -> effort 控制关节目标”的桥接结构

## 今日收获

### 1. 纯 PD 与逆动力学建模的效果差异非常明显
- 只靠关节空间 `PD` 控制时，系统虽然可以通过增益调大逼近目标，但容易持续摇摆和抖动。
- 加上逆动力学建模后，控制器不再只靠误差“硬拉”，而是显式考虑：
  - 惯性项
  - 速度耦合项
  - 重力项
- 实际效果表现为：
  - 到点更准确
  - 运动更丝滑
  - 抖动和摇摆基本消失

### 2. 问题根源不只是 PD 参数，而是动力学模型是否完整
- 之前 inverse dynamics 效果不明显，并不是控制思路本身错误。
- 更核心的问题是主 `robot.urdf` 缺少活动连杆的惯性参数，导致 Pinocchio 无法正确计算动力学项。
- 补齐动力学参数后，`rnea(...)` 的效果才真正体现出来。

### 3. 更清楚地区分了“底层通用日志”和“ROS2 节点日志”
- `robot_core` 中很多类不是 ROS2 节点类，没有 `get_logger()` 这样的节点上下文
- 因此底层通用日志器不应直接建立在 `RCLCPP_*` 上
- `robot_utils` 更适合作为“日志内核 / 通用工具层”
- `robot_ros` 这类节点层包则适合在上层使用统一宏，把打印行为接到同一套底层日志器

### 4. 理解了节流日志和最小日志等级的意义
- 节流日志本质是：
  - 对同一个日志调用点按时间窗口限流
  - 避免高频循环条件成立时刷屏
- 最小日志等级本质是：
  - 通过 `min_level` 过滤低等级日志
  - 平时隐藏 `Debug`
  - 调试时再打开 `Debug`

### 5. 对后续力控制路线更明确了
- 当前项目已经把：
  - 位姿控制
  - 姿态控制
  - effort 输出
  - inverse dynamics
  跑顺了
- 现在可以进入力控制篇章，但更合适的第一步不是纯力控制，而是：
  - 笛卡尔空间阻抗控制
- 同时进一步理清了：
  - 运动控制和力控制关注的主目标不同
  - 同一方向上通常不能同时刚性地独立指定位置和力
  - 接触约束、多目标和限幅问题后续很自然会和 QP 求解联系起来

## 当前遗留问题
- 当前 inverse dynamics 已经显著改善了到点性能和稳定性，后续还可以继续观察：
  - 不同轨迹速度下的稳定性
  - 是否还需要额外积分项
  - 是否需要把当前有效参数单独固化留档
- `robot_utils` 已经具备日志配置文件加载能力，但还没有自动在各 `main.cc` 启动阶段统一调用 `loadConfigFromFile()`。
- 目前 `robot_ros` 虽已切换到统一日志接口，但 `robot_core` 还没有逐步迁移到同一套日志器。
- 力控制部分还处在理论梳理阶段，下一步更可能先从笛卡尔空间阻抗控制入手。
