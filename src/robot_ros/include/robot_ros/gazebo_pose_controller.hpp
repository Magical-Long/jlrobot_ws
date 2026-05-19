#pragma once

/**
 * @file gazebo_pose_controller.hpp
 * @brief 定义用于 Gazebo 位置控制的目标位姿到关节轨迹桥接节点。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_core/model/robot_model.hpp"
#include "robot_core/planning/ik/ik_solver.hpp"
#include "robot_core/planning/trajectory/trajectory_planner.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace robot_ros
{
/**
 * @brief 将目标末端位姿转换成 Gazebo 可执行关节轨迹的 ROS2 节点。
 *
 * 节点职责：
 * 1. 订阅目标位姿 `geometry_msgs::msg::Pose`；
 * 2. 调用 `robot_core::IKSolver` 求解目标关节角；
 * 3. 以当前 `/joint_states` 为起点构造一段平滑关节轨迹；
 * 4. 发布 `trajectory_msgs::msg::JointTrajectory` 到 `arm_controller`。
 *
 * 对上层来说，这个节点就像一个简单的“位姿控制 API”：
 * - 输入：目标位姿话题；
 * - 输出：关节轨迹命令话题。
 */
class GazeboPoseControllerNode : public rclcpp::Node
{
public:
  /// @brief 构造并初始化 Gazebo 位姿控制节点。
  GazeboPoseControllerNode();

private:
  /// @brief 声明并读取节点参数。
  void declareParameters();

  /// @brief 初始化底层机器人模型、IK 和轨迹规划器。
  void initializeControllers();

  /// @brief 解析默认 URDF 路径。
  std::string resolveDefaultUrdfPath() const;

  /// @brief 处理来自 Gazebo 的当前关节状态。
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief 处理上层发来的目标末端位姿。
  void handleTargetPose(const geometry_msgs::msg::Pose::SharedPtr msg);

  /// @brief 根据当前关节状态和目标关节角生成轨迹消息。
  trajectory_msgs::msg::JointTrajectory buildTrajectoryMessage(
    const Eigen::VectorXd & start,
    const Eigen::VectorXd & goal) const;

  /// @brief 根据控制模式准备实际用于 IK 的目标位姿。
  geometry_msgs::msg::Pose buildIkTargetPose(
    const geometry_msgs::msg::Pose & requested_pose,
    const Eigen::VectorXd & start_configuration) const;

  /// @brief 用多组初值尝试求解目标位姿，提高 Gazebo 控制时的收敛鲁棒性。
  bool solveTargetConfiguration(
    const geometry_msgs::msg::Pose & requested_pose,
    const Eigen::VectorXd & start_configuration,
    Eigen::VectorXd & target_configuration);

  /// @brief 将内部轨迹点转换成 ROS `JointTrajectory` 消息。
  trajectory_msgs::msg::JointTrajectory toJointTrajectoryMessage(
    const std::vector<robot_core::JointTrajectoryPoint> & trajectory) const;

  /// @brief 检查并缓存最新的当前关节配置。
  bool updateCurrentConfigurationFromJointState(const sensor_msgs::msg::JointState & msg);

  /// @brief 机器人模型封装。
  std::shared_ptr<robot_core::RobotModel> robot_model_;

  /// @brief 逆运动学求解器。
  std::unique_ptr<robot_core::IKSolver> ik_solver_;

  /// @brief 关节空间轨迹规划器。
  std::unique_ptr<robot_core::JointTrajectoryPlanner> trajectory_planner_;

  /// @brief 目标位姿订阅器，上层通过这个接口输入控制目标。
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;

  /// @brief 当前关节状态订阅器，用于把 Gazebo 当前姿态作为轨迹起点。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  /// @brief 轨迹命令发布器，直接连到 `arm_controller`。
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  /// @brief 最新的当前关节配置。
  Eigen::VectorXd current_configuration_;

  /// @brief 是否已经收到可用于控制的当前关节状态。
  bool has_current_configuration_{false};

  /// @brief 节点加载模型时使用的 URDF 路径。
  std::string urdf_path_;

  /// @brief 当前模型使用的末端 frame 名称。
  std::string end_effector_frame_;

  /// @brief 目标位姿输入话题。
  std::string target_pose_topic_;

  /// @brief 关节状态输入话题。
  std::string joint_state_topic_;

  /// @brief 轨迹命令输出话题。
  std::string command_topic_;

  /// @brief 每段点到点轨迹的执行时长，单位为秒。
  double segment_duration_{3.0};

  /// @brief 轨迹离散采样周期，单位为秒。
  double sample_period_{0.02};

  /// @brief 是否只优先满足位置，并保持当前末端姿态不变。
  bool position_only_{true};
};
}  // namespace robot_ros
