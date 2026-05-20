#pragma once

/**
 * @file gazebo_admittance_controller.hpp
 * @brief 定义 Gazebo 下的最小三维位置导纳外环节点。
 */

#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_ros
{
/**
 * @brief 把末端外力转换成修正目标位姿的导纳外环节点。
 *
 * 这个节点位于用户目标位姿和现有位姿跟踪内环之间：
 * 1. 订阅原始目标位姿 `/target_pose`；
 * 2. 订阅末端 FT 传感器发布的 `/ee_wrench`；
 * 3. 用三维平移导纳方程把外力转换成位移偏移；
 * 4. 发布修正后的 `/admittance_target_pose` 给下游 IK / 轨迹 / 关节控制内环。
 *
 * 第一版为了先看出效果，只做：
 * - 平移方向导纳
 * - 姿态保持原始目标姿态不变
 * - 原始 wrench 零偏标定 + 死区 + 位移限幅
 */
class GazeboAdmittanceControllerNode : public rclcpp::Node
{
public:
  /// @brief 构造并初始化导纳外环节点。
  GazeboAdmittanceControllerNode();

private:
  /// @brief 声明并读取 ROS 参数。
  void declareParameters();

  /// @brief 处理来自上层的原始目标位姿。
  void handleTargetPose(const geometry_msgs::msg::Pose::SharedPtr msg);

  /// @brief 处理末端 FT 传感器的 wrench 反馈。
  void handleWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  /// @brief 周期执行一次导纳积分并发布修正后的目标位姿。
  void runAdmittanceLoop();

  /// @brief 对原始力做零偏去除和死区处理。
  Eigen::Vector3d computeFilteredForce() const;

  /// @brief 从参数数组解析出三维向量。
  Eigen::Vector3d vectorFromParameter(
    const std::vector<double> & values,
    const std::string & parameter_name) const;

  /// 原始目标位姿输入话题。
  std::string input_target_pose_topic_;

  /// 导纳修正后的目标位姿输出话题。
  std::string output_target_pose_topic_;

  /// FT 传感器 wrench 输入话题。
  std::string wrench_topic_;

  /// 导纳外环运行频率。
  double control_rate_hz_{200.0};

  /// 是否启用导纳控制；关闭时直接透传原始目标位姿。
  bool enable_admittance_{true};

  /// 导纳平移质量参数，对应 xyz 三个方向。
  Eigen::Vector3d admittance_mass_{Eigen::Vector3d::Constant(2.0)};

  /// 导纳平移阻尼参数，对应 xyz 三个方向。
  Eigen::Vector3d admittance_damping_{Eigen::Vector3d::Constant(80.0)};

  /// 导纳平移刚度参数，对应 xyz 三个方向。
  Eigen::Vector3d admittance_stiffness_{Eigen::Vector3d::Constant(120.0)};

  /// 末端外力死区，小于该阈值时视为 0。
  Eigen::Vector3d force_deadband_{Eigen::Vector3d::Constant(0.5)};

  /// 相对原始目标位姿允许的最大平移偏移。
  Eigen::Vector3d max_translation_offset_{Eigen::Vector3d::Constant(0.10)};

  /// 是否启用启动阶段的零偏自动标定。
  bool enable_wrench_bias_calibration_{true};

  /// 用于零偏标定的采样数量。
  int wrench_bias_sample_count_{200};

  /// 当前是否已经收到原始目标位姿。
  bool has_raw_target_pose_{false};

  /// 当前是否已经收到至少一帧 wrench。
  bool has_wrench_{false};

  /// 当前是否已经完成 wrench 零偏标定。
  bool has_wrench_bias_{false};

  /// 最近一次收到的原始目标位姿。
  geometry_msgs::msg::Pose raw_target_pose_;

  /// 最近一次收到的原始 FT 力数据。
  Eigen::Vector3d latest_raw_force_{Eigen::Vector3d::Zero()};

  /// 最近一次收到的原始 FT 力矩数据。
  Eigen::Vector3d latest_raw_torque_{Eigen::Vector3d::Zero()};

  /// 启动阶段估计得到的力零偏。
  Eigen::Vector3d wrench_force_bias_{Eigen::Vector3d::Zero()};

  /// 启动阶段估计得到的力矩零偏。
  Eigen::Vector3d wrench_torque_bias_{Eigen::Vector3d::Zero()};

  /// 零偏标定过程中的力累积和。
  Eigen::Vector3d wrench_force_bias_sum_{Eigen::Vector3d::Zero()};

  /// 零偏标定过程中的力矩累积和。
  Eigen::Vector3d wrench_torque_bias_sum_{Eigen::Vector3d::Zero()};

  /// 已用于零偏标定的样本数。
  int wrench_bias_samples_collected_{0};

  /// 当前导纳外环产生的平移偏移量。
  Eigen::Vector3d translation_offset_{Eigen::Vector3d::Zero()};

  /// 当前导纳外环产生的平移偏移速度。
  Eigen::Vector3d translation_offset_velocity_{Eigen::Vector3d::Zero()};

  /// 原始目标位姿订阅器。
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;

  /// FT 传感器 wrench 订阅器。
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;

  /// 修正后目标位姿发布器。
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr admittance_target_pose_publisher_;

  /// 导纳外环定时器。
  rclcpp::TimerBase::SharedPtr admittance_timer_;
};
}  // namespace robot_ros
