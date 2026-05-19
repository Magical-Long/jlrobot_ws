#include "robot_ros/gazebo_effort_controller.hpp"

/**
 * @file gazebo_effort_controller.cc
 * @brief 实现 Gazebo effort 模式下的关节空间 PD 力矩控制节点。
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_utils/logging.hpp"

namespace robot_ros
{
namespace
{
constexpr const char * kLogTag = "EFFORT";
}

GazeboEffortControllerNode::GazeboEffortControllerNode()
: rclcpp::Node("joint_effort_controller")
{
  // 先读取参数，再初始化模型和控制器。
  declareParameters();
  initializeController();

  // 订阅当前状态与目标参考，这样控制律和 ROS/Gazebo 接线就彻底解耦。
  joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    rclcpp::QoS(10),
    std::bind(&GazeboEffortControllerNode::handleJointState, this, std::placeholders::_1));

  reference_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
    reference_topic_,
    rclcpp::QoS(10),
    std::bind(&GazeboEffortControllerNode::handleReferenceJointState, this, std::placeholders::_1));

  torque_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    command_topic_,
    rclcpp::QoS(10));

  // effort 控制更像实时闭环，因此使用固定频率定时器周期执行。
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_))),
    std::bind(&GazeboEffortControllerNode::runControlLoop, this));

  ROBOT_UTILS_LOG_INFO_TAG(
    kLogTag,
    "Gazebo effort controller is ready. joint_state_topic=%s, reference_topic=%s, command_topic=%s",
    joint_state_topic_.c_str(),
    reference_topic_.c_str(),
    command_topic_.c_str());
}

void GazeboEffortControllerNode::declareParameters()
{
  urdf_path_ = declare_parameter<std::string>("urdf_path", resolveDefaultUrdfPath());
  end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "iiwa_link_ee");
  joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  reference_topic_ = declare_parameter<std::string>(
    "reference_topic", "/desired_joint_states");
  command_topic_ = declare_parameter<std::string>(
    "command_topic", "/arm_effort_controller/commands");
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 250.0);
  hold_current_position_on_startup_ =
    declare_parameter<bool>("hold_current_position_on_startup", true);
  enable_velocity_filter_ =
    declare_parameter<bool>("enable_velocity_filter", true);
  enable_velocity_filter_debug_topics_ =
    declare_parameter<bool>("enable_velocity_filter_debug_topics", true);
  raw_velocity_debug_topic_ =
    declare_parameter<std::string>("raw_velocity_debug_topic", "/debug/raw_joint_velocity");
  filtered_velocity_debug_topic_ =
    declare_parameter<std::string>("filtered_velocity_debug_topic", "/debug/filtered_joint_velocity");
  velocity_filter_type_ =
    declare_parameter<std::string>("velocity_filter_type", "low_pass");
  velocity_low_pass_cutoff_hz_ =
    declare_parameter<double>("velocity_low_pass_cutoff_hz", 12.0);
  velocity_one_euro_min_cutoff_hz_ =
    declare_parameter<double>("velocity_one_euro_min_cutoff_hz", 2.0);
  velocity_one_euro_beta_ =
    declare_parameter<double>("velocity_one_euro_beta", 0.02);
  velocity_one_euro_derivative_cutoff_hz_ =
    declare_parameter<double>("velocity_one_euro_derivative_cutoff_hz", 8.0);

  if (control_rate_hz_ <= 0.0)
  {
    throw std::runtime_error("Parameter 'control_rate_hz' must be greater than zero.");
  }
}

void GazeboEffortControllerNode::initializeController()
{
  robot_model_ = std::make_shared<robot_core::RobotModel>(urdf_path_, end_effector_frame_);
  if (!robot_model_->loadURDF())
  {
    throw std::runtime_error("Failed to load robot model from URDF: " + urdf_path_);
  }

  const Eigen::Index dof = static_cast<Eigen::Index>(robot_model_->dof());

  // 默认给一组各关节相同的 Kp/Kd，再允许用户通过参数覆盖。
  std::vector<double> default_kp(static_cast<std::size_t>(dof), 80.0);
  std::vector<double> default_kd(static_cast<std::size_t>(dof), 12.0);
  std::vector<double> default_max_torque(static_cast<std::size_t>(dof), 150.0);

  const std::vector<double> kp_values = declare_parameter("kp", default_kp);
  const std::vector<double> kd_values = declare_parameter("kd", default_kd);
  const bool enable_inverse_dynamics =
    declare_parameter<bool>("enable_inverse_dynamics", false);
  const bool enable_gravity_compensation =
    declare_parameter<bool>("enable_gravity_compensation", true);
  const bool enable_torque_limits =
    declare_parameter<bool>("enable_torque_limits", true);
  const std::vector<double> max_torque_values =
    declare_parameter("max_torque", default_max_torque);

  if (kp_values.size() != static_cast<std::size_t>(dof) ||
      kd_values.size() != static_cast<std::size_t>(dof) ||
      max_torque_values.size() != static_cast<std::size_t>(dof))
  {
    throw std::runtime_error("PD gains and torque limits must match the robot degrees of freedom.");
  }

  robot_core::JointSpacePDOptions options;
  options.kp = Eigen::Map<const Eigen::VectorXd>(kp_values.data(), dof);
  options.kd = Eigen::Map<const Eigen::VectorXd>(kd_values.data(), dof);
  options.enable_inverse_dynamics = enable_inverse_dynamics;
  options.enable_gravity_compensation = enable_gravity_compensation;
  options.enable_torque_limits = enable_torque_limits;
  options.max_torque = Eigen::Map<const Eigen::VectorXd>(max_torque_values.data(), dof);

  pd_controller_ = std::make_unique<robot_core::JointSpacePDController>(robot_model_, options);

  current_state_.positions = robot_model_->neutralConfiguration();
  current_state_.velocities = Eigen::VectorXd::Zero(dof);
  current_raw_velocities_ = Eigen::VectorXd::Zero(dof);

  reference_.positions = robot_model_->neutralConfiguration();
  reference_.velocities = Eigen::VectorXd::Zero(dof);
  reference_.accelerations = Eigen::VectorXd::Zero(dof);

  // 速度滤波放在 joint_states 入口处，目的是先把噪声抑制掉，
  // 再让 `Kd * (dq_des - dq)` 这一项参与力矩计算。
  //
  // 当前默认优先启用固定截止频率低通滤波，
  // 因为它对“零点附近速度噪声较大”的场景更直接、更容易调试。
  // One Euro Filter 的实现仍然完整保留在 `robot_core/filters/filters.*` 中，
  // 后续若要对比“更低滞后”的自适应滤波效果，只需把类型改回 `one_euro`。
  joint_velocity_filters_.clear();
  if (enable_velocity_filter_)
  {
    joint_velocity_filters_.reserve(static_cast<std::size_t>(dof));
    for (Eigen::Index idx = 0; idx < dof; ++idx)
    {
      joint_velocity_filters_.push_back(
        robot_core::createScalarFilter(
          velocity_filter_type_,
          velocity_low_pass_cutoff_hz_,
          velocity_one_euro_min_cutoff_hz_,
          velocity_one_euro_beta_,
          velocity_one_euro_derivative_cutoff_hz_));
    }
  }

  // ------------------------------ DEBUG BEGIN ------------------------------
  // 这两路调试话题专门用来对照“滤波前速度”和“滤波后速度”，
  // 典型使用方式是在 PlotJuggler 里把两条曲线叠在一起观察噪声抑制与相位滞后。
  // 如果后续不再需要调试滤波效果，可以连同参数和发布函数一起删除。
  if (enable_velocity_filter_debug_topics_)
  {
    raw_velocity_debug_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      raw_velocity_debug_topic_,
      rclcpp::QoS(10));
    filtered_velocity_debug_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      filtered_velocity_debug_topic_,
      rclcpp::QoS(10));
  }
  // ------------------------------- DEBUG END -------------------------------

  ROBOT_UTILS_LOG_INFO_TAG(
    kLogTag,
    "Joint torque controller configuration: inverse_dynamics=%s, gravity_compensation=%s, torque_limits=%s, "
    "velocity_filter=%s, velocity_filter_type=%s, velocity_filter_debug=%s",
    enable_inverse_dynamics ? "true" : "false",
    enable_gravity_compensation ? "true" : "false",
    enable_torque_limits ? "true" : "false",
    enable_velocity_filter_ ? "true" : "false",
    velocity_filter_type_.c_str(),
    enable_velocity_filter_debug_topics_ ? "true" : "false");
}

std::string GazeboEffortControllerNode::resolveDefaultUrdfPath() const
{
  return ament_index_cpp::get_package_share_directory("robot_description") + "/urdf/robot.urdf";
}

void GazeboEffortControllerNode::handleJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  if (!updateControlStateFromJointState(*msg))
  {
    return;
  }

  has_state_ = true;

  // ------------------------------ DEBUG BEGIN ------------------------------
  // 在同一帧 joint_states 处理完成后，同时把原始速度和滤波后速度发出去，
  // 这样 PlotJuggler 中两条曲线的时间戳天然对齐。
  publishVelocityDebugMessages(msg->header);
  // ------------------------------- DEBUG END -------------------------------

  // 若未显式给目标，就先把“当前状态”当作保持目标，
  // 这样 effort 模式启动后不会因为 zero torque 直接塌下去。
  if (hold_current_position_on_startup_ && !has_reference_)
  {
    reference_.positions = current_state_.positions;
    reference_.velocities = Eigen::VectorXd::Zero(current_state_.velocities.size());
    reference_.accelerations = Eigen::VectorXd::Zero(current_state_.velocities.size());
    has_reference_ = true;

    ROBOT_UTILS_LOG_INFO_TAG(
      kLogTag,
      "No external reference received yet. Holding the current joint configuration as the startup target.");
  }
}

void GazeboEffortControllerNode::handleReferenceJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  if (!updateReferenceFromJointState(*msg))
  {
    ROBOT_UTILS_LOG_WARN_TAG(kLogTag, "Ignoring invalid desired_joint_states message.");
    return;
  }

  has_reference_ = true;
  ROBOT_UTILS_LOG_INFO_TAG(
    kLogTag,
    "Updated PD torque control reference from %s.",
    reference_topic_.c_str());
}

void GazeboEffortControllerNode::runControlLoop()
{
  if (!has_state_ || !has_reference_)
  {
    return;
  }

  Eigen::VectorXd torque_command;
  if (!pd_controller_->computeTorque(current_state_, reference_, torque_command))
  {
    ROBOT_UTILS_LOG_WARN_THROTTLE_MS_TAG(
      kLogTag,
      2000,
      "Skipping one effort control cycle because controller input is invalid.");
    return;
  }

  torque_publisher_->publish(toTorqueCommandMessage(torque_command));
}

bool GazeboEffortControllerNode::updateControlStateFromJointState(
  const sensor_msgs::msg::JointState & msg)
{
  const std::vector<std::string> & expected_joint_names = robot_model_->jointNames();
  if (msg.name.empty() || msg.position.empty())
  {
    return false;
  }

  const Eigen::Index dof = static_cast<Eigen::Index>(robot_model_->dof());
  current_state_.positions = Eigen::VectorXd::Zero(dof);
  current_state_.velocities = Eigen::VectorXd::Zero(dof);
  current_raw_velocities_ = Eigen::VectorXd::Zero(dof);
  const double filter_dt = resolveVelocityFilterDt(msg);

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

    current_state_.positions[static_cast<Eigen::Index>(joint_idx)] = msg.position[array_index];
    if (array_index < msg.velocity.size())
    {
      const double raw_velocity = msg.velocity[array_index];
      current_raw_velocities_[static_cast<Eigen::Index>(joint_idx)] = raw_velocity;
      double measured_velocity = raw_velocity;

      // 若启用了滤波，则对每个关节的速度反馈分别做时序平滑。
      // 这样既能减少 Gazebo 数值噪声，又不会把所有关节耦合成一条公共滤波状态。
      if (enable_velocity_filter_ &&
          joint_idx < joint_velocity_filters_.size() &&
          joint_velocity_filters_[joint_idx])
      {
        measured_velocity =
          joint_velocity_filters_[joint_idx]->filter(measured_velocity, filter_dt);
      }

      current_state_.velocities[static_cast<Eigen::Index>(joint_idx)] = measured_velocity;
    }
  }

  return true;
}

bool GazeboEffortControllerNode::updateReferenceFromJointState(
  const sensor_msgs::msg::JointState & msg)
{
  const std::vector<std::string> & expected_joint_names = robot_model_->jointNames();
  if (msg.name.empty() || msg.position.empty())
  {
    return false;
  }

  const Eigen::Index dof = static_cast<Eigen::Index>(robot_model_->dof());
  reference_.positions = Eigen::VectorXd::Zero(dof);
  reference_.velocities = Eigen::VectorXd::Zero(dof);
  reference_.accelerations = Eigen::VectorXd::Zero(dof);

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

    reference_.positions[static_cast<Eigen::Index>(joint_idx)] = msg.position[array_index];
    if (array_index < msg.velocity.size())
    {
      reference_.velocities[static_cast<Eigen::Index>(joint_idx)] = msg.velocity[array_index];
    }
  }

  return true;
}

void GazeboEffortControllerNode::publishVelocityDebugMessages(const std_msgs::msg::Header & header)
{
  if (!enable_velocity_filter_debug_topics_)
  {
    return;
  }

  const std::vector<std::string> & joint_names = robot_model_->jointNames();

  if (raw_velocity_debug_publisher_)
  {
    sensor_msgs::msg::JointState raw_message;
    raw_message.header = header;
    raw_message.name = joint_names;
    raw_message.velocity.assign(
      current_raw_velocities_.data(),
      current_raw_velocities_.data() + current_raw_velocities_.size());
    raw_velocity_debug_publisher_->publish(raw_message);
  }

  if (filtered_velocity_debug_publisher_)
  {
    sensor_msgs::msg::JointState filtered_message;
    filtered_message.header = header;
    filtered_message.name = joint_names;
    filtered_message.velocity.assign(
      current_state_.velocities.data(),
      current_state_.velocities.data() + current_state_.velocities.size());
    filtered_velocity_debug_publisher_->publish(filtered_message);
  }
}

double GazeboEffortControllerNode::resolveVelocityFilterDt(const sensor_msgs::msg::JointState & msg)
{
  // 优先使用 joint_states 自带时间戳，这样滤波时间轴和实际反馈采样更一致。
  rclcpp::Time current_stamp(msg.header.stamp);
  if (current_stamp.nanoseconds() == 0)
  {
    // 若消息未填 header.stamp，则退回到本节点当前时间，避免 dt 恒为零。
    current_stamp = now();
  }

  double dt = 1.0 / std::max(1.0, control_rate_hz_);
  if (has_last_joint_state_stamp_)
  {
    dt = (current_stamp - last_joint_state_stamp_).seconds();
  }

  last_joint_state_stamp_ = current_stamp;
  has_last_joint_state_stamp_ = true;

  // 对异常小或异常大的 dt 做保底，避免一条坏时间戳把滤波器状态打坏。
  if (dt <= 1.0e-6 || dt > 0.5)
  {
    return 1.0 / std::max(1.0, control_rate_hz_);
  }

  return dt;
}

std_msgs::msg::Float64MultiArray GazeboEffortControllerNode::toTorqueCommandMessage(
  const Eigen::VectorXd & torque_command) const
{
  std_msgs::msg::Float64MultiArray message;
  message.data.assign(torque_command.data(), torque_command.data() + torque_command.size());
  return message;
}
}  // namespace robot_ros
