#include "robot_ros/gazebo_pose_controller.hpp"

/**
 * @file gazebo_pose_controller.cc
 * @brief 实现将目标末端位姿转成 Gazebo 关节轨迹命令的桥接节点。
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robot_ros
{
  GazeboPoseControllerNode::GazeboPoseControllerNode()
      : rclcpp::Node("gazebo_pose_controller")
  {
    // 先把所有参数读进来，后续模型、话题和控制器初始化都依赖这些配置。
    declareParameters();
    initializeControllers();

    // 当前状态来自 Gazebo / ros2_control 的 joint_states，
    // 目标位姿来自上层算法或测试脚本。
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_,
        rclcpp::QoS(10),
        std::bind(&GazeboPoseControllerNode::handleJointState, this, std::placeholders::_1));

    target_pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
        target_pose_topic_,
        rclcpp::QoS(10),
        std::bind(&GazeboPoseControllerNode::handleTargetPose, this, std::placeholders::_1));

    // 轨迹命令直接发给 Gazebo 中已经激活的 `arm_controller`。
    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        command_topic_,
        rclcpp::QoS(10));

    RCLCPP_INFO(
        get_logger(),
        "Gazebo pose controller is ready. target_pose_topic=%s, joint_state_topic=%s, command_topic=%s",
        target_pose_topic_.c_str(),
        joint_state_topic_.c_str(),
        command_topic_.c_str());
  }

  void GazeboPoseControllerNode::declareParameters()
  {
    // 上层最关心的是：模型从哪加载、目标从哪进来、轨迹发到哪去。
    urdf_path_ = declare_parameter<std::string>("urdf_path", resolveDefaultUrdfPath());
    end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "iiwa_link_ee");
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/target_pose");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    command_topic_ = declare_parameter<std::string>(
        "command_topic", "/arm_controller/joint_trajectory");
    segment_duration_ = declare_parameter<double>("segment_duration", 3.0);
    sample_period_ = declare_parameter<double>("sample_period", 0.02);
    position_only_ = declare_parameter<bool>("position_only", true);

    if (segment_duration_ <= 0.0)
    {
      throw std::runtime_error("Parameter 'segment_duration' must be greater than zero.");
    }
    if (sample_period_ <= 0.0)
    {
      throw std::runtime_error("Parameter 'sample_period' must be greater than zero.");
    }
  }

  void GazeboPoseControllerNode::initializeControllers()
  {
    // 用和 `robot_core` 一致的方式加载模型，确保 IK / 轨迹规划 / Gazebo 使用同一套关节定义。
    robot_model_ = std::make_shared<robot_core::RobotModel>(urdf_path_, end_effector_frame_);
    if (!robot_model_->loadURDF())
    {
      throw std::runtime_error("Failed to load robot model from URDF: " + urdf_path_);
    }

    robot_core::IKOptions ik_options;
    // Gazebo 控制更强调“能稳定到位”，这里把默认 IK 参数调得更宽松一些。
    const std::string solver_method =
        declare_parameter<std::string>("ik_solver_method", "lm");
    ik_options.solver_method =
        (solver_method == "lm")
            ? robot_core::IKSolverMethod::LevenbergMarquardt
            : robot_core::IKSolverMethod::DampedLeastSquares;
    // 在线交互控制更看重“快速给出成功/失败反馈”，
    // 因此默认迭代次数不宜过大，避免不可达点长时间卡住回调线程。
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

    // 在收到真实 joint_states 前，先用中性位姿占位，便于日志和调试。
    current_configuration_ = robot_model_->neutralConfiguration();
  }

  std::string GazeboPoseControllerNode::resolveDefaultUrdfPath() const
  {
    // Gazebo 控制节点默认读取描述包里的基础 URDF，
    // 因为 IK / FK 关心的是机器人运动学本体，而不是 Gazebo 插件附加字段。
    return ament_index_cpp::get_package_share_directory("robot_description") + "/urdf/robot.urdf";
  }

  void GazeboPoseControllerNode::handleJointState(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    if (updateCurrentConfigurationFromJointState(*msg))
    {
      has_current_configuration_ = true;
    }
  }

  void GazeboPoseControllerNode::handleTargetPose(
      const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    // 在仿真中优先使用当前真实关节状态作为轨迹起点，
    // 这样不会每次都默认从中性位姿重新规划。
    const Eigen::VectorXd start_configuration = has_current_configuration_
                                                    ? current_configuration_
                                                    : robot_model_->neutralConfiguration();

    if (!has_current_configuration_)
    {
      RCLCPP_WARN(
          get_logger(),
          "No valid joint state has been received yet. Falling back to neutral configuration as IK seed.");
    }

    Eigen::VectorXd target_configuration;
    if (!solveTargetConfiguration(*msg, start_configuration, target_configuration))
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
    trajectory_msgs::msg::JointTrajectory command;
    try
    {
      command = buildTrajectoryMessage(start_configuration, target_configuration);
    }
    catch (const std::exception &exception)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to build command trajectory: %s",
          exception.what());
      return;
    }
    trajectory_publisher_->publish(command);

    RCLCPP_INFO(
        get_logger(),
        "Published %zu trajectory points to %s for target pose [%.4f, %.4f, %.4f].",
        command.points.size(),
        command_topic_.c_str(),
        msg->position.x,
        msg->position.y,
        msg->position.z);
  }

  trajectory_msgs::msg::JointTrajectory GazeboPoseControllerNode::buildTrajectoryMessage(
      const Eigen::VectorXd &start,
      const Eigen::VectorXd &goal) const
  {
    std::vector<robot_core::JointTrajectoryPoint> trajectory;
    const std::vector<Eigen::VectorXd> waypoints{start, goal};

    if (!trajectory_planner_->planWaypoints(
            waypoints, segment_duration_, trajectory, sample_period_))
    {
      throw std::runtime_error("Failed to generate joint trajectory from IK solution.");
    }

    return toJointTrajectoryMessage(trajectory);
  }

  geometry_msgs::msg::Pose GazeboPoseControllerNode::buildIkTargetPose(
      const geometry_msgs::msg::Pose &requested_pose,
      const Eigen::VectorXd &start_configuration) const
  {
    if (!position_only_)
    {
      return requested_pose;
    }

    geometry_msgs::msg::Pose ik_target_pose = requested_pose;
    geometry_msgs::msg::Pose current_pose;
    if (robot_model_->forwardKinematics(start_configuration, current_pose))
    {
      // 位置控制模式下，保留当前末端姿态，只要求末端移动到目标位置。
      ik_target_pose.orientation = current_pose.orientation;
    }

    return ik_target_pose;
  }

  bool GazeboPoseControllerNode::solveTargetConfiguration(
      const geometry_msgs::msg::Pose &requested_pose,
      const Eigen::VectorXd &start_configuration,
      Eigen::VectorXd &target_configuration)
  {
    std::vector<Eigen::VectorXd> seeds;
    seeds.reserve(5);
    seeds.push_back(robot_model_->clampToLimits(start_configuration));
    seeds.push_back(robot_model_->neutralConfiguration());

    // 再补几组已知可达的关节姿态作为兜底初值，
    // 避免数值 IK 因当前姿态局部条件不好而卡死。
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
      RCLCPP_INFO(
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
        RCLCPP_INFO(
            get_logger(),
            "IK solved with seed #%zu in %.2f ms.",
            idx + 1,
            elapsed_ms);
        return true;
      }

      RCLCPP_WARN(
          get_logger(),
          "IK seed #%zu failed after %.2f ms.",
          idx + 1,
          elapsed_ms);
    }

    return false;
  }

  trajectory_msgs::msg::JointTrajectory GazeboPoseControllerNode::toJointTrajectoryMessage(
      const std::vector<robot_core::JointTrajectoryPoint> &trajectory) const
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.header.stamp = now();
    message.joint_names = robot_model_->jointNames();

    for (const auto &point : trajectory)
    {
      trajectory_msgs::msg::JointTrajectoryPoint ros_point;

      ros_point.positions.assign(
          point.positions.data(),
          point.positions.data() + point.positions.size());
      ros_point.velocities.assign(
          point.velocities.data(),
          point.velocities.data() + point.velocities.size());
      ros_point.accelerations.assign(
          point.accelerations.data(),
          point.accelerations.data() + point.accelerations.size());
      ros_point.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start);
      message.points.push_back(ros_point);
    }

    return message;
  }

  bool GazeboPoseControllerNode::updateCurrentConfigurationFromJointState(
      const sensor_msgs::msg::JointState &msg)
  {
    const std::vector<std::string> &expected_joint_names = robot_model_->jointNames();
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

      const std::size_t position_index = static_cast<std::size_t>(
          std::distance(msg.name.begin(), name_it));
      if (position_index >= msg.position.size())
      {
        return false;
      }

      configuration[static_cast<Eigen::Index>(joint_idx)] = msg.position[position_index];
    }

    if (!robot_model_->isConfigurationValid(configuration))
    {
      configuration = robot_model_->clampToLimits(configuration);
    }

    current_configuration_ = configuration;
    return true;
  }
} // namespace robot_ros
