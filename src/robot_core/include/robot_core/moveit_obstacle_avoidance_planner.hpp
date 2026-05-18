#pragma once

/**
 * @file moveit_obstacle_avoidance_planner.hpp
 * @brief 定义基于 MoveIt 2 的机械臂避障规划器接口。
 *
 * 这个规划器和当前基于势场法的 `ObstacleAvoidancePlanner` 不同：
 * 1. 它把碰撞检测和采样规划交给 MoveIt 2；
 * 2. 障碍物会作为 `CollisionObject` 注入规划场景；
 * 3. 输出既保留关节路径点，也缓存 MoveIt 生成的 `JointTrajectory`。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_core/obstacle_avoidance_planner.hpp"

namespace moveit
{
namespace core
{
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace moveit_cpp
{
class MoveItCpp;
class PlanningComponent;
}  // namespace moveit_cpp

namespace robot_core
{
/**
 * @brief MoveIt 2 避障规划参数。
 *
 * 注意这些参数需要和你的 MoveIt SRDF / OMPL 配置保持一致，
 * 尤其是 `planning_group`、`end_effector_link`、`planning_pipeline` 和 `planner_id`。
 */
struct MoveItObstacleAvoidanceOptions
{
  /// MoveIt 规划组名称，例如常见的 `manipulator`。
  std::string planning_group{"manipulator"};

  /// 末端执行器 link 名称，需与 SRDF / URDF 中配置一致。
  std::string end_effector_link{"iiwa_link_ee"};

  /// 目标位姿与障碍物默认所在的规划参考坐标系。
  std::string planning_frame{"world"};

  /// MoveIt 规划管线名称，常见为 `ompl`。
  std::string planning_pipeline{"ompl"};

  /// 具体规划器 id，例如 `RRTConnectkConfigDefault`。
  std::string planner_id{"RRTConnectkConfigDefault"};

  /// 单次规划允许使用的最长时间，单位为秒。
  double planning_time{5.0};

  /// 单次目标的规划尝试次数。
  int planning_attempts{5};

  /// 轨迹最大速度缩放因子，范围通常为 `(0, 1]`。
  double max_velocity_scaling_factor{0.2};

  /// 轨迹最大加速度缩放因子，范围通常为 `(0, 1]`。
  double max_acceleration_scaling_factor{0.2};

  /// 目标位置容差，单位为米。
  double goal_position_tolerance{0.01};

  /// 目标姿态容差，单位为弧度。
  double goal_orientation_tolerance{0.05};

  /// 是否要求同时约束末端姿态。
  bool hold_orientation{true};
};

/**
 * @brief 基于 MoveIt 2 规划场景的机械臂避障规划器。
 *
 * 推荐使用流程：
 * 1. 在节点中创建该类实例；
 * 2. 调用 `initialize()` 启动 MoveItCpp 和 PlanningSceneMonitor；
 * 3. 对每个目标位姿调用 `planToPose()`；
 * 4. 如需下发控制，可读取 `lastJointTrajectory()`。
 */
class MoveItObstacleAvoidancePlanner
{
public:
  /**
   * @brief 构造规划器。
   *
   * @param node ROS2 节点，用于初始化 MoveItCpp。
   * @param options MoveIt 规划参数。
   */
  explicit MoveItObstacleAvoidancePlanner(
    const rclcpp::Node::SharedPtr & node,
    const MoveItObstacleAvoidanceOptions & options = MoveItObstacleAvoidanceOptions{});

  /// 因为类中持有前置声明类型的智能指针，所以析构放到源文件里定义。
  ~MoveItObstacleAvoidancePlanner();

  /**
   * @brief 初始化 MoveItCpp 与规划组件。
   *
   * 如果已经初始化过，该函数会直接返回 true。
   *
   * @return true 初始化成功。
   * @return false 初始化失败。
   */
  bool initialize();

  /**
   * @brief 使用 MoveIt 2 规划一条避障关节路径。
   *
   * @param target_pose 目标末端位姿。
   * @param obstacles 工作空间球形障碍物集合。
   * @param joint_waypoints 输出关节路径点。
   * @param initial_guess 可选起始关节配置；若为空则使用当前场景状态。
   * @return true 规划成功并输出至少一个路径点。
   * @return false 初始化失败、目标不可达或规划失败。
   */
  bool planToPose(const geometry_msgs::msg::Pose & target_pose,
                  const std::vector<SphericalObstacle> & obstacles,
                  std::vector<Eigen::VectorXd> & joint_waypoints,
                  const Eigen::VectorXd & initial_guess = Eigen::VectorXd());

  /**
   * @brief 获取当前规划参数。
   *
   * @return const MoveItObstacleAvoidanceOptions& 当前参数。
   */
  const MoveItObstacleAvoidanceOptions & options() const;

  /**
   * @brief 替换当前规划参数。
   *
   * @param options 新参数。
   */
  void setOptions(const MoveItObstacleAvoidanceOptions & options);

  /**
   * @brief 获取最近一次规划得到的 `JointTrajectory`。
   *
   * @return const trajectory_msgs::msg::JointTrajectory& 最近一次 MoveIt 规划轨迹。
   */
  const trajectory_msgs::msg::JointTrajectory & lastJointTrajectory() const;

private:
  /// 将外部传入的球形障碍物同步到 MoveIt 规划场景。
  bool applyObstaclesToPlanningScene(const std::vector<SphericalObstacle> & obstacles);

  /// 把用户给定的起始关节配置写入 MoveIt 的 start state。
  bool applyStartState(const Eigen::VectorXd & initial_guess);

  /// 从 MoveIt 返回的 `JointTrajectory` 中提取纯关节位置路径点。
  bool extractJointWaypoints(std::vector<Eigen::VectorXd> & joint_waypoints) const;

  /// 解析当前规划使用的世界参考坐标系名称。
  std::string resolvePlanningFrame() const;

  /// 上层注入的 ROS2 节点，MoveItCpp 和日志都依赖它。
  rclcpp::Node::SharedPtr node_;

  /// 当前规划器使用的 MoveIt 配置快照。
  MoveItObstacleAvoidanceOptions options_;

  /// 标记 `initialize()` 是否已经成功完成，避免重复启动监视器。
  bool initialized_{false};

  /// MoveItCpp 总入口，内部封装了 robot model、planning scene 和规划管线。
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;

  /// 面向单个 planning group 的便捷规划接口。
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;

  /// 当前 planning group 在 MoveIt robot model 中对应的关节组描述。
  const moveit::core::JointModelGroup * joint_model_group_{nullptr};

  /// 记录当前已经注入场景的障碍物 id，便于下一次规划前先删除旧对象。
  std::vector<std::string> active_collision_object_ids_;

  /// 缓存最近一次 MoveIt 规划出的完整关节轨迹消息。
  trajectory_msgs::msg::JointTrajectory last_joint_trajectory_;
};
}  // namespace robot_core
