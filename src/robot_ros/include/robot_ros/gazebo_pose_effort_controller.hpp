#pragma once

/**
 * @file gazebo_pose_effort_controller.hpp
 * @brief 定义“末端位姿目标 -> effort 控制所需关节目标轨迹”桥接节点。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_core/ik_solver.hpp"
#include "robot_core/robot_model.hpp"
#include "robot_core/trajectory_planner.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_ros
{
/**
 * @brief 将末端位姿目标转换成 effort 控制链路所需关节目标序列的 ROS2 节点。
 *
 * 该节点位于“任务空间层”和“关节力矩控制层”之间，职责如下：
 * 1. 订阅目标末端位姿 `/target_pose`；
 * 2. 读取当前 `/joint_states` 作为 IK 初值和轨迹起点；
 * 3. 调用 `robot_core::IKSolver` 求解目标关节角；
 * 4. 调用 `robot_core::JointTrajectoryPlanner` 生成平滑关节目标轨迹；
 * 5. 周期发布 `sensor_msgs::msg::JointState` 到 `/desired_joint_states`。
 *
 * 这样下层 `gazebo_effort_controller` 只关心关节目标，不需要知道末端位姿、IK 和轨迹规划细节。
 */
class GazeboPoseEffortControllerNode : public rclcpp::Node
{
public:
  /// @brief 构造并初始化“位姿目标 -> effort 关节目标”的桥接节点。
  GazeboPoseEffortControllerNode();

private:
  /// @brief 声明并读取节点参数。
  void declareParameters();

  /// @brief 初始化机器人模型、IK 求解器和轨迹规划器。
  void initializeControllers();

  /// @brief 解析默认 URDF 路径。
  std::string resolveDefaultUrdfPath() const;

  /// @brief 处理当前关节状态反馈。
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief 处理目标末端位姿请求。
  void handleTargetPose(const geometry_msgs::msg::Pose::SharedPtr msg);

  /// @brief 定时发布当前活动关节目标轨迹中的下一个采样点。
  void publishNextReferencePoint();

  /// @brief 根据起止关节角生成内部关节目标轨迹。
  bool buildReferenceTrajectory(const Eigen::VectorXd & start, const Eigen::VectorXd & goal);

  /// @brief 多 seed 求解目标位姿对应的关节角。
  bool solveTargetConfiguration(
    const geometry_msgs::msg::Pose & requested_pose,
    const Eigen::VectorXd & start_configuration,
    Eigen::VectorXd & target_configuration,
    std::size_t * solved_seed_index = nullptr,
    double * solve_time_ms = nullptr);

  /// @brief 根据控制模式构造实际用于 IK 的目标位姿。
  geometry_msgs::msg::Pose buildIkTargetPose(
    const geometry_msgs::msg::Pose & requested_pose,
    const Eigen::VectorXd & start_configuration) const;

  /// @brief 从 JointState 中提取并缓存当前关节角。
  bool updateCurrentConfigurationFromJointState(const sensor_msgs::msg::JointState & msg);

  /// @brief 判断新的目标位姿是否与上一次已接受目标近似相同。
  bool isSameTargetPose(const geometry_msgs::msg::Pose & pose) const;

  /// @brief 发布当前真实末端位姿，供调试坐标系与跟踪误差使用。
  void publishDebugEndEffectorPose();

  /// @brief 把内部轨迹点转换成发给下层 effort 控制器的 JointState 目标。
  sensor_msgs::msg::JointState toReferenceJointState(
    const robot_core::JointTrajectoryPoint & point) const;

  /// @brief 底层机器人模型。
  std::shared_ptr<robot_core::RobotModel> robot_model_;

  /// @brief 逆运动学求解器。
  std::unique_ptr<robot_core::IKSolver> ik_solver_;

  /// @brief 关节轨迹插值器。
  std::unique_ptr<robot_core::JointTrajectoryPlanner> trajectory_planner_;

  /// @brief 目标位姿订阅器。
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;

  /// @brief 关节状态订阅器。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  /// @brief 关节目标发布器，输出给 `/desired_joint_states`。
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;

  /// @brief 调试用末端位姿发布器。
  /// 这不是闭环控制必需链路，只是为了把当前 `iiwa_link_ee` 的真实世界位姿直接打到话题上，
  /// 方便和 `/target_pose` 对照。后续若不再需要调试，可连同相关参数与函数一起删除。
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_ee_pose_publisher_;

  /// @brief 定时器，用于按固定采样周期发送内部关节目标轨迹。
  rclcpp::TimerBase::SharedPtr reference_timer_;

  /// @brief 当前真实关节角。
  Eigen::VectorXd current_configuration_;

  /// @brief 是否已经收到有效 joint_states。
  bool has_current_configuration_{false};

  /// @brief 是否已经缓存过上一次接受的目标位姿。
  bool has_last_target_pose_{false};

  /// @brief 上一次已接受的目标位姿，用于抑制重复输入刷屏。
  geometry_msgs::msg::Pose last_target_pose_;

  /// @brief 当前活动关节目标轨迹。
  std::vector<robot_core::JointTrajectoryPoint> active_reference_trajectory_;

  /// @brief 当前发布到轨迹的第几个采样点。
  std::size_t active_reference_index_{0};

  /// @brief 是否在只控制末端位置的模式下求解 IK。
  bool position_only_{true};

  /// @brief 每段点到点轨迹时长。
  double segment_duration_{3.0};

  /// @brief 参考轨迹采样周期。
  double sample_period_{0.02};

  /// @brief 判断两次目标位置是否视作同一目标的阈值，单位米。
  double repeated_target_position_tolerance_{1.0e-6};

  /// @brief 判断两次目标姿态是否视作同一目标的阈值。
  double repeated_target_orientation_tolerance_{1.0e-6};

  /// @brief 模型加载路径。
  std::string urdf_path_;

  /// @brief 末端 frame 名称。
  std::string end_effector_frame_;

  /// @brief 目标位姿输入话题。
  std::string target_pose_topic_;

  /// @brief 当前关节状态输入话题。
  std::string joint_state_topic_;

  /// @brief 下发给关节 effort 控制器的关节目标话题。
  std::string reference_topic_;

  /// @brief 是否启用末端位姿调试发布。
  bool enable_debug_ee_pose_publisher_{true};

  /// @brief 末端位姿调试话题名。
  std::string debug_ee_pose_topic_;
};
}  // namespace robot_ros
