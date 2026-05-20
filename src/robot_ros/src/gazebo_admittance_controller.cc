#include "robot_ros/gazebo_admittance_controller.hpp"

/**
 * @file gazebo_admittance_controller.cc
 * @brief 实现 Gazebo 下的最小三维位置导纳外环节点。
 */

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "robot_utils/logging.hpp"

namespace robot_ros
{
  namespace
  {
    constexpr const char *kLogTag = "ADMIT";
  }

  GazeboAdmittanceControllerNode::GazeboAdmittanceControllerNode()
      : rclcpp::Node("gazebo_admittance_controller")
  {
    // 先把参数读出来，再启动 ROS 通信和外环定时器。
    declareParameters();

    // 原始目标位姿来自用户或上层规划器。
    target_pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
        input_target_pose_topic_,
        rclcpp::QoS(10),
        std::bind(&GazeboAdmittanceControllerNode::handleTargetPose, this, std::placeholders::_1));

    // 外力反馈来自 Gazebo FT 传感器链。
    wrench_subscriber_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_topic_,
        rclcpp::QoS(10),
        std::bind(&GazeboAdmittanceControllerNode::handleWrench, this, std::placeholders::_1));

    // 对下游内环来说，这里输出的仍然只是一个“普通目标位姿”。
    admittance_target_pose_publisher_ = create_publisher<geometry_msgs::msg::Pose>(
        output_target_pose_topic_,
        rclcpp::QoS(10));

    admittance_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_))),
        std::bind(&GazeboAdmittanceControllerNode::runAdmittanceLoop, this));

    ROBOT_UTILS_LOG_INFO_TAG(
        kLogTag,
        "Gazebo admittance controller is ready. input_target_pose_topic=%s, wrench_topic=%s, output_target_pose_topic=%s",
        input_target_pose_topic_.c_str(),
        wrench_topic_.c_str(),
        output_target_pose_topic_.c_str());
  }

  void GazeboAdmittanceControllerNode::declareParameters()
  {
    input_target_pose_topic_ =
        declare_parameter<std::string>("input_target_pose_topic", "/target_pose");
    output_target_pose_topic_ =
        declare_parameter<std::string>("output_target_pose_topic", "/admittance_target_pose");
    wrench_topic_ =
        declare_parameter<std::string>("wrench_topic", "/ee_wrench");
    control_rate_hz_ =
        declare_parameter<double>("control_rate_hz", 200.0);
    enable_admittance_ =
        declare_parameter<bool>("enable_admittance", true);

    admittance_mass_ = vectorFromParameter(
        declare_parameter<std::vector<double>>(
            "admittance_mass", std::vector<double>{2.0, 2.0, 2.0}),
        "admittance_mass");
    admittance_damping_ = vectorFromParameter(
        declare_parameter<std::vector<double>>(
            "admittance_damping", std::vector<double>{80.0, 80.0, 80.0}),
        "admittance_damping");
    admittance_stiffness_ = vectorFromParameter(
        declare_parameter<std::vector<double>>(
            "admittance_stiffness", std::vector<double>{120.0, 120.0, 120.0}),
        "admittance_stiffness");
    force_deadband_ = vectorFromParameter(
        declare_parameter<std::vector<double>>(
            "force_deadband", std::vector<double>{0.5, 0.5, 0.5}),
        "force_deadband");
    max_translation_offset_ = vectorFromParameter(
        declare_parameter<std::vector<double>>(
            "max_translation_offset", std::vector<double>{0.10, 0.10, 0.10}),
        "max_translation_offset");

    enable_wrench_bias_calibration_ =
        declare_parameter<bool>("enable_wrench_bias_calibration", true);
    wrench_bias_sample_count_ =
        declare_parameter<int>("wrench_bias_sample_count", 200);

    if (control_rate_hz_ <= 0.0)
    {
      throw std::runtime_error("Parameter 'control_rate_hz' must be greater than zero.");
    }

    if (wrench_bias_sample_count_ < 1)
    {
      throw std::runtime_error("Parameter 'wrench_bias_sample_count' must be at least 1.");
    }
  }

  void GazeboAdmittanceControllerNode::handleTargetPose(
      const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    // 用户原始目标位姿本身不改，只把它当成导纳平衡点。
    raw_target_pose_ = *msg;
    has_raw_target_pose_ = true;

    ROBOT_UTILS_LOG_INFO_TAG(
        kLogTag,
        "Updated raw target pose to [%.4f, %.4f, %.4f].",
        msg->position.x,
        msg->position.y,
        msg->position.z);
  }

  void GazeboAdmittanceControllerNode::handleWrench(
      const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    // 第一版导纳只使用三维力；力矩先缓存起来但不进入平移导纳方程。
    latest_raw_force_.x() = msg->wrench.force.x;
    latest_raw_force_.y() = msg->wrench.force.y;
    latest_raw_force_.z() = msg->wrench.force.z;
    latest_raw_torque_.x() = msg->wrench.torque.x;
    latest_raw_torque_.y() = msg->wrench.torque.y;
    latest_raw_torque_.z() = msg->wrench.torque.z;
    has_wrench_ = true;

    // 导纳外环若直接吃原始 wrench，静止噪声会让目标自己慢慢漂。
    // 因此这里在启动阶段先自动估计一份静态零偏，后续统一减掉它。
    if (enable_wrench_bias_calibration_ && !has_wrench_bias_)
    {
      wrench_force_bias_sum_ += latest_raw_force_;
      wrench_torque_bias_sum_ += latest_raw_torque_;
      ++wrench_bias_samples_collected_;

      if (wrench_bias_samples_collected_ >= wrench_bias_sample_count_)
      {
        wrench_force_bias_ =
            wrench_force_bias_sum_ / static_cast<double>(wrench_bias_samples_collected_);
        wrench_torque_bias_ =
            wrench_torque_bias_sum_ / static_cast<double>(wrench_bias_samples_collected_);
        has_wrench_bias_ = true;

        ROBOT_UTILS_LOG_INFO_TAG(
            kLogTag,
            "Completed wrench bias calibration. force_bias=[%.5f, %.5f, %.5f], torque_bias=[%.5f, %.5f, %.5f]",
            wrench_force_bias_.x(),
            wrench_force_bias_.y(),
            wrench_force_bias_.z(),
            wrench_torque_bias_.x(),
            wrench_torque_bias_.y(),
            wrench_torque_bias_.z());
      }
    }
    else if (!enable_wrench_bias_calibration_)
    {
      // 若显式关闭自动标定，就把 bias 固定为零并视为已就绪。
      has_wrench_bias_ = true;
    }
  }

  void GazeboAdmittanceControllerNode::runAdmittanceLoop()
  {
    if (!has_raw_target_pose_)
    {
      return;
    }

    geometry_msgs::msg::Pose commanded_pose = raw_target_pose_;

    // 若未启用导纳，或者 wrench 还没准备好，就直接透传原始目标位姿。
    if (!enable_admittance_ || !has_wrench_ || !has_wrench_bias_)
    {
      admittance_target_pose_publisher_->publish(commanded_pose);
      return;
    }

    const double dt = 1.0 / std::max(1.0, control_rate_hz_);
    const Eigen::Vector3d filtered_force = computeFilteredForce();

    // 三维平移导纳方程：
    //
    //   M * x_ddot + D * x_dot + K * x = F_ext
    //
    // 这里的 `x` 不是绝对末端位置，而是“相对原始目标位姿的偏移量”。
    // 因此当外力为 0 时，偏移量会在虚拟弹簧阻尼作用下回到 0。
    const Eigen::Vector3d translation_offset_acceleration =
        (filtered_force - admittance_damping_.cwiseProduct(translation_offset_velocity_) - admittance_stiffness_.cwiseProduct(translation_offset_))
            .cwiseQuotient(admittance_mass_);

    // 用最简单的显式欧拉积分先把效果跑起来，后续若有需要再升级积分器。
    translation_offset_velocity_ += translation_offset_acceleration * dt;
    translation_offset_ += translation_offset_velocity_ * dt;

    // 为了防止初始参数较激进时目标偏移过大，这里先做轴向限幅。
    translation_offset_ =
        translation_offset_.cwiseMin(max_translation_offset_).cwiseMax(-max_translation_offset_);

    // 若位置偏移打到了限幅边界，再把对应方向速度清掉，避免持续“顶着边界积分”。
    for (int idx = 0; idx < 3; ++idx)
    {
      if (translation_offset_[idx] >= max_translation_offset_[idx] && translation_offset_velocity_[idx] > 0.0)
      {
        translation_offset_velocity_[idx] = 0.0;
      }
      if (translation_offset_[idx] <= -max_translation_offset_[idx] && translation_offset_velocity_[idx] < 0.0)
      {
        translation_offset_velocity_[idx] = 0.0;
      }
    }

    // 第一版只修正平移，姿态完全沿用原始目标姿态。
    commanded_pose.position.x += translation_offset_.x();
    commanded_pose.position.y += translation_offset_.y();
    commanded_pose.position.z += translation_offset_.z();

    admittance_target_pose_publisher_->publish(commanded_pose);
  }

  Eigen::Vector3d GazeboAdmittanceControllerNode::computeFilteredForce() const
  {
    // 先去掉启动阶段估计得到的静态零偏。
    Eigen::Vector3d corrected_force = latest_raw_force_ - wrench_force_bias_;

    // 再施加死区，避免原始噪声把导纳外环慢慢推跑。
    for (int idx = 0; idx < 3; ++idx)
    {
      if (std::abs(corrected_force[idx]) < force_deadband_[idx])
      {
        corrected_force[idx] = 0.0;
      }
    }

    return corrected_force;
  }

  Eigen::Vector3d GazeboAdmittanceControllerNode::vectorFromParameter(
      const std::vector<double> &values,
      const std::string &parameter_name) const
  {
    if (values.size() != 3U)
    {
      throw std::runtime_error(
          "Parameter '" + parameter_name + "' must contain exactly 3 values.");
    }

    return Eigen::Vector3d(values[0], values[1], values[2]);
  }
} // namespace robot_ros
