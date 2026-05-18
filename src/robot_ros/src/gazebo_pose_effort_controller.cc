#include "robot_ros/gazebo_pose_effort_controller.hpp"

/**
 * @file gazebo_pose_effort_controller.cc
 * @brief 实现“位姿目标 -> effort 控制所需关节目标轨迹”桥接节点。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robot_ros
{
GazeboPoseEffortControllerNode::GazeboPoseEffortControllerNode()
: rclcpp::Node("pose_effort_controller")
{
  // 先读取参数，再初始化模型和算法对象。
  declareParameters();
  initializeControllers();

  // 当前关节状态来自 Gazebo / ros2_control，目标位姿来自上层任务输入。
  joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    rclcpp::QoS(10),
    std::bind(&GazeboPoseEffortControllerNode::handleJointState, this, std::placeholders::_1));

  target_pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
    target_pose_topic_,
    rclcpp::QoS(10),
    std::bind(&GazeboPoseEffortControllerNode::handleTargetPose, this, std::placeholders::_1));

  // 下层 effort 控制器只认关节目标，因此这里统一发布 JointState。
  reference_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    reference_topic_,
    rclcpp::QoS(10));

  // ------------------------------ DEBUG BEGIN ------------------------------
  // 这一段专门用于调试“控制器认为的真实末端位姿”。
  // 它不参与任何 IK、轨迹规划或力矩控制，只是把当前 `iiwa_link_ee`
  // 的世界系 PoseStamped 发出来，方便和 `/target_pose` 做直接对比。
  //
  // 如果后续这条调试链路不再需要，可以安全删除：
  // 1. 这里的 publisher 初始化；
  // 2. 头文件中的 debug publisher / 参数成员；
  // 3. `publishDebugEndEffectorPose()` 函数；
  // 4. `handleJointState()` 里对该函数的调用。
  if (enable_debug_ee_pose_publisher_)
  {
    debug_ee_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      debug_ee_pose_topic_,
      rclcpp::QoS(10));
  }
  // ------------------------------- DEBUG END -------------------------------

  // 按轨迹采样周期依次发送关节目标点，而不是一次性把终点关节角丢给下层。
  reference_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sample_period_)),
    std::bind(&GazeboPoseEffortControllerNode::publishNextReferencePoint, this));

  RCLCPP_INFO(
    get_logger(),
    "Gazebo pose effort controller is ready. target_pose_topic=%s, joint_state_topic=%s, effort_target_topic=%s",
    target_pose_topic_.c_str(),
    joint_state_topic_.c_str(),
    reference_topic_.c_str());

  if (enable_debug_ee_pose_publisher_)
  {
    RCLCPP_INFO(
      get_logger(),
      "Debug ee pose publisher is enabled. debug_ee_pose_topic=%s, end_effector_frame=%s",
      debug_ee_pose_topic_.c_str(),
      end_effector_frame_.c_str());
  }
}

void GazeboPoseEffortControllerNode::declareParameters()
{
  urdf_path_ = declare_parameter<std::string>("urdf_path", resolveDefaultUrdfPath());
  end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "iiwa_link_ee");
  target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/target_pose");
  joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  reference_topic_ = declare_parameter<std::string>("reference_topic", "/desired_joint_states");
  enable_debug_ee_pose_publisher_ =
    declare_parameter<bool>("enable_debug_ee_pose_publisher", true);
  debug_ee_pose_topic_ =
    declare_parameter<std::string>("debug_ee_pose_topic", "/debug/current_ee_pose");
  segment_duration_ = declare_parameter<double>("segment_duration", 3.0);
  sample_period_ = declare_parameter<double>("sample_period", 0.02);
  position_only_ = declare_parameter<bool>("position_only", true);
  repeated_target_position_tolerance_ =
    declare_parameter<double>("repeated_target_position_tolerance", 1.0e-6);
  repeated_target_orientation_tolerance_ =
    declare_parameter<double>("repeated_target_orientation_tolerance", 1.0e-6);

  if (segment_duration_ <= 0.0)
  {
    throw std::runtime_error("Parameter 'segment_duration' must be greater than zero.");
  }
  if (sample_period_ <= 0.0)
  {
    throw std::runtime_error("Parameter 'sample_period' must be greater than zero.");
  }
}

void GazeboPoseEffortControllerNode::initializeControllers()
{
  // 让 IK、轨迹规划和下层控制共享同一套机器人运动学定义。
  robot_model_ = std::make_shared<robot_core::RobotModel>(urdf_path_, end_effector_frame_);
  if (!robot_model_->loadURDF())
  {
    throw std::runtime_error("Failed to load robot model from URDF: " + urdf_path_);
  }

  robot_core::IKOptions ik_options;
  const std::string solver_method =
    declare_parameter<std::string>("ik_solver_method", "lm");
  ik_options.solver_method =
    (solver_method == "lm")
    ? robot_core::IKSolverMethod::LevenbergMarquardt
    : robot_core::IKSolverMethod::DampedLeastSquares;
  ik_options.max_iterations = declare_parameter<int>("ik_max_iterations", 200);
  ik_options.position_tolerance = declare_parameter<double>("ik_position_tolerance", 5.0e-3);
  ik_options.orientation_tolerance = declare_parameter<double>("ik_orientation_tolerance", 2.0e-1);
  ik_options.damping = declare_parameter<double>("ik_damping", 1.0e-3);
  ik_options.step_size = declare_parameter<double>("ik_step_size", 0.15);
  ik_options.enable_joint_limit_nullspace =
    declare_parameter<bool>("enable_joint_limit_nullspace", true);
  ik_options.joint_limit_nullspace_gain =
    declare_parameter<double>("joint_limit_nullspace_gain", 0.03);

  ik_solver_ = std::make_unique<robot_core::IKSolver>(robot_model_, ik_options);
  trajectory_planner_ =
    std::make_unique<robot_core::JointTrajectoryPlanner>(robot_model_, sample_period_);

  // 在第一次收到真实状态前，用中性位姿占位，便于算法对象完成初始化。
  current_configuration_ = robot_model_->neutralConfiguration();
}

std::string GazeboPoseEffortControllerNode::resolveDefaultUrdfPath() const
{
  return ament_index_cpp::get_package_share_directory("robot_description") + "/urdf/robot.urdf";
}

void GazeboPoseEffortControllerNode::handleJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  if (updateCurrentConfigurationFromJointState(*msg))
  {
    has_current_configuration_ = true;

    // ------------------------------ DEBUG BEGIN ------------------------------
    // 调试模式下，每次收到真实 joint_states 都同步发布一次真实末端位姿。
    // 这样看到的是“机器人现在实际在哪里”，而不是“目标想让它去哪里”。
    publishDebugEndEffectorPose();
    // ------------------------------- DEBUG END -------------------------------
  }
}

void GazeboPoseEffortControllerNode::handleTargetPose(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  Eigen::VectorXd start_configuration = has_current_configuration_
    ? current_configuration_
    : robot_model_->neutralConfiguration();

  if (!has_current_configuration_)
  {
    RCLCPP_WARN(
      get_logger(),
      "No valid joint state has been received yet. Falling back to neutral configuration as IK seed.");
  }
  // Gazebo effort 模式下关节可能因重力或数值积分略微越过 URDF 限位。
  // 轨迹规划器会严格检查限位，因此这里先把轨迹起点夹回模型允许范围。
  start_configuration = robot_model_->clampToLimits(start_configuration);

  // 上层若以固定频率重复发送同一个目标位姿，这里直接复用当前目标，
  // 避免同一目标反复触发 IK 和轨迹重建，把终端日志刷得很乱。
  if (has_last_target_pose_ && isSameTargetPose(*msg))
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Ignoring repeated target pose [%.4f, %.4f, %.4f].",
      msg->position.x,
      msg->position.y,
      msg->position.z);
    return;
  }

  Eigen::VectorXd target_configuration;
  std::size_t solved_seed_index = 0;
  double solve_time_ms = 0.0;
  if (!solveTargetConfiguration(
        *msg,
        start_configuration,
        target_configuration,
        &solved_seed_index,
        &solve_time_ms))
  {
    geometry_msgs::msg::Pose current_pose;
    if (robot_model_->forwardKinematics(start_configuration, current_pose))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to solve IK for target pose [%.4f, %.4f, %.4f]. position_only=%s. "
        "Current ee pose is [%.4f, %.4f, %.4f].",
        msg->position.x,
        msg->position.y,
        msg->position.z,
        position_only_ ? "true" : "false",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z);
    }
    else
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to solve IK for target pose [%.4f, %.4f, %.4f]. position_only=%s",
        msg->position.x,
        msg->position.y,
        msg->position.z,
        position_only_ ? "true" : "false");
    }
    return;
  }

  target_configuration = robot_model_->clampToLimits(target_configuration);
  if (!buildReferenceTrajectory(start_configuration, target_configuration))
  {
    RCLCPP_ERROR(get_logger(), "Failed to build reference trajectory from IK solution.");
    return;
  }

  // 只有当本次目标真正被接受并生成了新轨迹后，才更新“上一次目标”缓存。
  last_target_pose_ = *msg;
  has_last_target_pose_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Accepted target pose [%.4f, %.4f, %.4f]. IK seed=%zu/%d, solve_time=%.2f ms, target_points=%zu, position_only=%s.",
    msg->position.x,
    msg->position.y,
    msg->position.z,
    solved_seed_index + 1,
    5,
    solve_time_ms,
    active_reference_trajectory_.size(),
    position_only_ ? "true" : "false");
}

void GazeboPoseEffortControllerNode::publishNextReferencePoint()
{
  if (active_reference_trajectory_.empty())
  {
    return;
  }

  if (active_reference_index_ >= active_reference_trajectory_.size())
  {
    // 已经发送到轨迹末尾后，保持最后一个参考点不再递增索引。
    active_reference_index_ = active_reference_trajectory_.size() - 1;
  }

  reference_publisher_->publish(
    toReferenceJointState(active_reference_trajectory_[active_reference_index_]));

  if (active_reference_index_ + 1 < active_reference_trajectory_.size())
  {
    ++active_reference_index_;
  }
}

bool GazeboPoseEffortControllerNode::buildReferenceTrajectory(
  const Eigen::VectorXd & start,
  const Eigen::VectorXd & goal)
{
  if (!robot_model_->isConfigurationValid(start))
  {
    RCLCPP_WARN(
      get_logger(),
      "Start configuration from joint_states is slightly outside URDF limits. Clamping it before trajectory generation.");
  }

  const Eigen::VectorXd clamped_start = robot_model_->clampToLimits(start);
  const Eigen::VectorXd clamped_goal = robot_model_->clampToLimits(goal);

  std::vector<robot_core::JointTrajectoryPoint> trajectory;
  const std::vector<Eigen::VectorXd> waypoints{clamped_start, clamped_goal};

  if (!trajectory_planner_->planWaypoints(
        waypoints, segment_duration_, trajectory, sample_period_))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Trajectory planner rejected the IK result. start_size=%ld goal_size=%ld dof=%zu "
      "start_valid=%s goal_valid=%s segment_duration=%.4f sample_period=%.4f",
      static_cast<long>(clamped_start.size()),
      static_cast<long>(clamped_goal.size()),
      robot_model_->dof(),
      robot_model_->isConfigurationValid(clamped_start) ? "true" : "false",
      robot_model_->isConfigurationValid(clamped_goal) ? "true" : "false",
      segment_duration_,
      sample_period_);
    return false;
  }

  active_reference_trajectory_ = trajectory;
  active_reference_index_ = 0;
  return !active_reference_trajectory_.empty();
}

bool GazeboPoseEffortControllerNode::solveTargetConfiguration(
  const geometry_msgs::msg::Pose & requested_pose,
  const Eigen::VectorXd & start_configuration,
  Eigen::VectorXd & target_configuration,
  std::size_t * solved_seed_index,
  double * solve_time_ms)
{
  std::vector<Eigen::VectorXd> seeds;
  seeds.reserve(5);
  seeds.push_back(robot_model_->clampToLimits(start_configuration));
  seeds.push_back(robot_model_->neutralConfiguration());

  // 增加几组经验初值，提高数值 IK 的鲁棒性。
  Eigen::VectorXd sample_a = robot_model_->neutralConfiguration();
  sample_a[0] = -0.55;
  sample_a[1] = 0.85;
  sample_a[2] = 1.10;
  sample_a[3] = -0.95;
  sample_a[4] = 1.20;
  sample_a[5] = 0.80;
  sample_a[6] = -1.35;
  seeds.push_back(robot_model_->clampToLimits(sample_a));

  Eigen::VectorXd sample_b = robot_model_->neutralConfiguration();
  sample_b[0] = 0.20;
  sample_b[1] = 0.35;
  sample_b[3] = -0.45;
  sample_b[5] = 0.30;
  seeds.push_back(robot_model_->clampToLimits(sample_b));

  Eigen::VectorXd sample_c = robot_model_->neutralConfiguration();
  sample_c[0] = -0.25;
  sample_c[2] = 0.40;
  sample_c[4] = -0.30;
  sample_c[6] = 0.35;
  seeds.push_back(robot_model_->clampToLimits(sample_c));

  for (std::size_t idx = 0; idx < seeds.size(); ++idx)
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Trying IK seed #%zu/%zu for target [%.4f, %.4f, %.4f]. position_only=%s",
      idx + 1,
      seeds.size(),
      requested_pose.position.x,
      requested_pose.position.y,
      requested_pose.position.z,
      position_only_ ? "true" : "false");

    const auto seed_start_time = std::chrono::steady_clock::now();
    Eigen::VectorXd candidate;
    const bool solved = position_only_
      ? ik_solver_->solvePositionOnly(requested_pose, candidate, seeds[idx])
      : ik_solver_->solve(
          buildIkTargetPose(requested_pose, start_configuration),
          candidate,
          seeds[idx]);
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - seed_start_time)
      .count();
    if (solved)
    {
      target_configuration = candidate;
      if (solved_seed_index)
      {
        *solved_seed_index = idx;
      }
      if (solve_time_ms)
      {
        *solve_time_ms = elapsed_ms;
      }
      return true;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "IK seed #%zu failed after %.2f ms.",
      idx + 1,
      elapsed_ms);
  }

  return false;
}

bool GazeboPoseEffortControllerNode::isSameTargetPose(const geometry_msgs::msg::Pose & pose) const
{
  const double dx = pose.position.x - last_target_pose_.position.x;
  const double dy = pose.position.y - last_target_pose_.position.y;
  const double dz = pose.position.z - last_target_pose_.position.z;
  const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

  const double dqx = pose.orientation.x - last_target_pose_.orientation.x;
  const double dqy = pose.orientation.y - last_target_pose_.orientation.y;
  const double dqz = pose.orientation.z - last_target_pose_.orientation.z;
  const double dqw = pose.orientation.w - last_target_pose_.orientation.w;
  const double orientation_error = std::sqrt(
    dqx * dqx + dqy * dqy + dqz * dqz + dqw * dqw);

  return position_error <= repeated_target_position_tolerance_ &&
         orientation_error <= repeated_target_orientation_tolerance_;
}

geometry_msgs::msg::Pose GazeboPoseEffortControllerNode::buildIkTargetPose(
  const geometry_msgs::msg::Pose & requested_pose,
  const Eigen::VectorXd & start_configuration) const
{
  if (!position_only_)
  {
    return requested_pose;
  }

  geometry_msgs::msg::Pose ik_target_pose = requested_pose;
  geometry_msgs::msg::Pose current_pose;
  if (robot_model_->forwardKinematics(start_configuration, current_pose))
  {
    // 位置模式下只改位置，姿态保持当前值，避免额外姿态约束降低可达性。
    ik_target_pose.orientation = current_pose.orientation;
  }

  return ik_target_pose;
}

bool GazeboPoseEffortControllerNode::updateCurrentConfigurationFromJointState(
  const sensor_msgs::msg::JointState & msg)
{
  const std::vector<std::string> & expected_joint_names = robot_model_->jointNames();
  if (msg.name.empty() || msg.position.empty())
  {
    return false;
  }

  Eigen::VectorXd configuration = robot_model_->neutralConfiguration();

  for (std::size_t joint_idx = 0; joint_idx < expected_joint_names.size(); ++joint_idx)
  {
    const auto name_it = std::find(
      msg.name.begin(), msg.name.end(), expected_joint_names[joint_idx]);
    if (name_it == msg.name.end())
    {
      return false;
    }

    const std::size_t array_index = static_cast<std::size_t>(
      std::distance(msg.name.begin(), name_it));
    if (array_index >= msg.position.size())
    {
      return false;
    }

    configuration[static_cast<Eigen::Index>(joint_idx)] = msg.position[array_index];
  }

  if (!robot_model_->isConfigurationValid(configuration))
  {
    configuration = robot_model_->clampToLimits(configuration);
  }

  current_configuration_ = configuration;

  return true;
}

void GazeboPoseEffortControllerNode::publishDebugEndEffectorPose()
{
  // ------------------------------ DEBUG BEGIN ------------------------------
  // 该函数只服务于调试：
  // - 输入：当前真实 joint_states 还原出的 `current_configuration_`
  // - 输出：当前 `iiwa_link_ee` 在世界坐标系下的真实位姿
  //
  // 如果未来调试结束，这整个函数都可以删除，不影响主控制逻辑。
  if (!enable_debug_ee_pose_publisher_ || !debug_ee_pose_publisher_ || !has_current_configuration_)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped debug_pose_message;
  debug_pose_message.header.stamp = now();
  // `RobotModel::forwardKinematics()` 返回的是 world -> ee 的位姿，
  // 因此这里明确标成 `world`，便于和 Gazebo 世界坐标对照。
  debug_pose_message.header.frame_id = "world";

  if (!robot_model_->forwardKinematics(current_configuration_, debug_pose_message.pose))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Failed to publish debug ee pose because forward kinematics on current joint state failed.");
    return;
  }

  debug_ee_pose_publisher_->publish(debug_pose_message);
  // ------------------------------- DEBUG END -------------------------------
}

sensor_msgs::msg::JointState GazeboPoseEffortControllerNode::toReferenceJointState(
  const robot_core::JointTrajectoryPoint & point) const
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = now();
  message.name = robot_model_->jointNames();
  message.position.assign(point.positions.data(), point.positions.data() + point.positions.size());
  message.velocity.assign(point.velocities.data(), point.velocities.data() + point.velocities.size());
  return message;
}
}  // namespace robot_ros
