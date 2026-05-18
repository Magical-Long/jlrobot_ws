#pragma once

/**
 * @file gazebo_effort_controller.hpp
 * @brief 定义 Gazebo effort 控制模式下的关节空间 PD 力矩控制节点。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "robot_core/control_reference.hpp"
#include "robot_core/control_state.hpp"
#include "robot_core/filters.hpp"
#include "robot_core/joint_space_pd_controller.hpp"
#include "robot_core/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_ros
{
/**
 * @brief Gazebo effort 控制桥接节点。
 *
 * 这个节点负责把 Gazebo 返回的关节状态和上层给出的目标关节参考接起来，
 * 然后调用 `robot_core::JointSpacePDController` 计算每个周期的关节力矩命令。
 *
 * 数据流如下：
 *
 * `joint_states -> ControlState -> PD Controller -> torque command -> /arm_effort_controller/commands`
 */
class GazeboEffortControllerNode : public rclcpp::Node
{
public:
  /// @brief 构造并初始化 Gazebo effort 控制节点。
  GazeboEffortControllerNode();

private:
  /// @brief 声明并读取 ROS 参数。
  void declareParameters();

  /// @brief 初始化机器人模型与 PD 控制器。
  void initializeController();

  /// @brief 解析默认 URDF 路径。
  std::string resolveDefaultUrdfPath() const;

  /// @brief 读取 Gazebo 返回的当前关节状态。
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief 读取外部给出的目标关节参考。
  void handleReferenceJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief 周期执行一次闭环控制并发布当前力矩命令。
  void runControlLoop();

  /// @brief 从 `JointState` 按模型关节顺序提取位置/速度到控制状态。
  bool updateControlStateFromJointState(const sensor_msgs::msg::JointState & msg);

  /// @brief 从 `JointState` 按模型关节顺序提取目标位置/速度到控制参考。
  bool updateReferenceFromJointState(const sensor_msgs::msg::JointState & msg);

  /// @brief 根据 joint_states 时间戳估算本次速度滤波的采样时间。
  double resolveVelocityFilterDt(const sensor_msgs::msg::JointState & msg);

  /// @brief 发布原始/滤波后关节速度，便于 PlotJuggler 对照滤波效果。
  void publishVelocityDebugMessages(const std_msgs::msg::Header & header);

  /// @brief 将 Eigen 力矩向量转换成 ROS `Float64MultiArray`。
  std_msgs::msg::Float64MultiArray toTorqueCommandMessage(
    const Eigen::VectorXd & torque_command) const;

  /// 底层机器人模型。
  std::shared_ptr<robot_core::RobotModel> robot_model_;

  /// 关节空间 PD 力矩控制器。
  std::unique_ptr<robot_core::JointSpacePDController> pd_controller_;

  /// 当前反馈状态。
  robot_core::ControlState current_state_;

  /// 调试用的原始关节速度。
  /// 这个向量保留的是 `joint_states` 里的原始速度顺序重排结果，
  /// 便于和滤波后的 `current_state_.velocities` 做一一对照。
  Eigen::VectorXd current_raw_velocities_;

  /// 当前目标参考。
  robot_core::ControlReference reference_;

  /// 是否已经收到有效当前状态。
  bool has_state_{false};

  /// 是否已经收到有效目标参考。
  bool has_reference_{false};

  /// 若为 true，则第一次收到状态后自动把当前姿态作为保持目标。
  bool hold_current_position_on_startup_{true};

  /// 当前使用的 URDF 路径。
  std::string urdf_path_;

  /// 当前末端 frame 名称。
  std::string end_effector_frame_;

  /// 关节状态输入话题。
  std::string joint_state_topic_;

  /// 目标关节参考输入话题。
  std::string reference_topic_;

  /// effort 控制器命令输出话题。
  std::string command_topic_;

  /// 控制循环频率。
  double control_rate_hz_{250.0};

  /// 是否启用关节速度滤波。
  bool enable_velocity_filter_{true};

  /// 当前选用的速度滤波器类型，可选 `none` / `low_pass` / `one_euro`。
  /// 当前默认优先使用固定截止频率低通，因为它对“零点附近速度噪声很大”的场景更直接。
  std::string velocity_filter_type_{"low_pass"};

  /// 一阶低通滤波截止频率，单位 Hz。
  double velocity_low_pass_cutoff_hz_{12.0};

  /// One Euro 的基础截止频率，单位 Hz。
  double velocity_one_euro_min_cutoff_hz_{2.0};

  /// One Euro 的速度相关增益。
  double velocity_one_euro_beta_{0.02};

  /// One Euro 导数低通截止频率，单位 Hz。
  double velocity_one_euro_derivative_cutoff_hz_{8.0};

  /// 是否启用滤波调试话题。
  bool enable_velocity_filter_debug_topics_{true};

  /// 原始关节速度调试话题。
  std::string raw_velocity_debug_topic_{"/debug/raw_joint_velocity"};

  /// 滤波后关节速度调试话题。
  std::string filtered_velocity_debug_topic_{"/debug/filtered_joint_velocity"};

  /// 每个关节各自独立的速度滤波器实例。
  std::vector<std::unique_ptr<robot_core::ScalarFilterBase>> joint_velocity_filters_;

  /// 是否已经有过用于滤波的上一帧 joint_states 时间戳。
  bool has_last_joint_state_stamp_{false};

  /// 上一帧 joint_states 对应的时间戳。
  rclcpp::Time last_joint_state_stamp_{0, 0, RCL_ROS_TIME};

  /// 关节状态订阅器。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  /// 目标参考订阅器。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_subscriber_;

  /// 力矩命令发布器。
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;

  /// 调试用原始关节速度发布器。
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_velocity_debug_publisher_;

  /// 调试用滤波后关节速度发布器。
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr filtered_velocity_debug_publisher_;

  /// 控制循环定时器。
  rclcpp::TimerBase::SharedPtr control_timer_;
};
}  // namespace robot_ros
