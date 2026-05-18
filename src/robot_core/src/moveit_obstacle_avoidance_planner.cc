#include "robot_core/moveit_obstacle_avoidance_planner.hpp"

/**
 * @file moveit_obstacle_avoidance_planner.cc
 * @brief 实现基于 MoveIt 2 PlanningScene 和 PlanningComponent 的避障规划逻辑。
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace robot_core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

moveit_msgs::msg::CollisionObject makeSphereCollisionObject(
  const SphericalObstacle & obstacle,
  const std::string & frame_id,
  const std::string & object_id)
{
  // 每个障碍物都会被转成一个独立的 MoveIt `CollisionObject`，
  // 这样便于后续按 id 精确删除或替换。
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = object_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // 当前用球体近似障碍物，因此 primitive 只需要填一个半径参数。
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  primitive.dimensions = {obstacle.radius + obstacle.safety_margin};
  object.primitives.push_back(primitive);

  // 球体姿态对碰撞几何没有影响，这里只需要设置中心位置和单位四元数。
  geometry_msgs::msg::Pose pose;
  pose.position.x = obstacle.center.x();
  pose.position.y = obstacle.center.y();
  pose.position.z = obstacle.center.z();
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);

  return object;
}
}  // namespace

MoveItObstacleAvoidancePlanner::MoveItObstacleAvoidancePlanner(
  const rclcpp::Node::SharedPtr & node,
  const MoveItObstacleAvoidanceOptions & options)
: node_(node), options_(options)
{
  if (!node_)
  {
    throw std::invalid_argument("MoveItObstacleAvoidancePlanner requires a valid ROS2 node.");
  }
}

MoveItObstacleAvoidancePlanner::~MoveItObstacleAvoidancePlanner() = default;

bool MoveItObstacleAvoidancePlanner::initialize()
{
  // 已初始化过时直接复用已有对象，避免重复拉起监视线程。
  if (initialized_)
  {
    return true;
  }

  // MoveItCpp 是 MoveIt 2 C++ 侧的总入口，负责统一装配场景、模型和规划组件。
  moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
  auto scene_monitor = moveit_cpp_->getPlanningSceneMonitorNonConst();
  if (!scene_monitor)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get PlanningSceneMonitor from MoveItCpp.");
    return false;
  }

  // 三个 monitor 分别负责：
  // - 对外提供 planning scene 服务
  // - 监听环境几何变化
  // - 监听机器人当前状态变化
  scene_monitor->providePlanningSceneService();
  scene_monitor->startSceneMonitor();
  scene_monitor->startWorldGeometryMonitor();
  scene_monitor->startStateMonitor();

  // `PlanningComponent` 是按 planning group 封装好的“单目标规划器”。
  planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
    options_.planning_group, moveit_cpp_);
  joint_model_group_ =
    moveit_cpp_->getRobotModel()->getJointModelGroup(options_.planning_group);

  if (!joint_model_group_)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveIt planning group '%s' was not found in the loaded robot model.",
      options_.planning_group.c_str());
    planning_component_.reset();
    moveit_cpp_.reset();
    return false;
  }

  initialized_ = true;
  return true;
}

bool MoveItObstacleAvoidancePlanner::planToPose(
  const geometry_msgs::msg::Pose & target_pose,
  const std::vector<SphericalObstacle> & obstacles,
  std::vector<Eigen::VectorXd> & joint_waypoints,
  const Eigen::VectorXd & initial_guess)
{
  joint_waypoints.clear();
  last_joint_trajectory_ = trajectory_msgs::msg::JointTrajectory();

  if (!initialize())
  {
    return false;
  }

  if (!applyObstaclesToPlanningScene(obstacles))
  {
    return false;
  }

  if (!applyStartState(initial_guess))
  {
    return false;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = resolvePlanningFrame();
  goal_pose.header.stamp = node_->now();
  goal_pose.pose = target_pose;

  // MoveIt 的 goal constraints 需要按 xyz 和 rpy 三个维度分别给容差。
  const std::vector<double> position_tolerances(
    3, std::max(options_.goal_position_tolerance, 1.0e-6));
  const std::vector<double> orientation_tolerances(
    3,
    options_.hold_orientation ?
    std::max(options_.goal_orientation_tolerance, 1.0e-6) :
    kPi);

  // 这里把目标位姿包装成 MoveIt 约束，让底层规划器在可行空间内寻找无碰路径。
  const moveit_msgs::msg::Constraints goal_constraints =
    kinematic_constraints::constructGoalConstraints(
    options_.end_effector_link,
    goal_pose,
    position_tolerances,
    orientation_tolerances);
  planning_component_->setGoal({goal_constraints});

  // 这组请求参数控制具体使用哪条管线、给多长时间、以及速度/加速度缩放比例。
  moveit_cpp::PlanningComponent::PlanRequestParameters request_parameters;
  request_parameters.planning_attempts = std::max(1, options_.planning_attempts);
  request_parameters.planning_pipeline = options_.planning_pipeline;
  request_parameters.planner_id = options_.planner_id;
  request_parameters.planning_time = std::max(0.1, options_.planning_time);
  request_parameters.max_velocity_scaling_factor =
    std::min(1.0, std::max(1.0e-3, options_.max_velocity_scaling_factor));
  request_parameters.max_acceleration_scaling_factor =
    std::min(1.0, std::max(1.0e-3, options_.max_acceleration_scaling_factor));

  auto plan_solution = planning_component_->plan(request_parameters);
  if (!plan_solution || !plan_solution.trajectory)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "MoveIt planning failed for group '%s' toward the requested target pose.",
      options_.planning_group.c_str());
    return false;
  }

  // MoveIt 内部先返回 `RobotTrajectory` 包装对象，这里再导出成标准 ROS 消息缓存下来。
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  plan_solution.trajectory->getRobotTrajectoryMsg(trajectory_msg);
  last_joint_trajectory_ = trajectory_msg.joint_trajectory;

  return extractJointWaypoints(joint_waypoints);
}

const MoveItObstacleAvoidanceOptions & MoveItObstacleAvoidancePlanner::options() const
{
  return options_;
}

void MoveItObstacleAvoidancePlanner::setOptions(const MoveItObstacleAvoidanceOptions & options)
{
  options_ = options;
}

const trajectory_msgs::msg::JointTrajectory &
MoveItObstacleAvoidancePlanner::lastJointTrajectory() const
{
  return last_joint_trajectory_;
}

bool MoveItObstacleAvoidancePlanner::applyObstaclesToPlanningScene(
  const std::vector<SphericalObstacle> & obstacles)
{
  auto scene_monitor = moveit_cpp_->getPlanningSceneMonitorNonConst();
  if (!scene_monitor)
  {
    return false;
  }

  planning_scene_monitor::LockedPlanningSceneRW scene(scene_monitor);
  const std::string planning_frame = resolvePlanningFrame();

  // 先把上一次规划留下的障碍物删干净，避免场景里不断累积旧对象。
  for (const std::string & object_id : active_collision_object_ids_)
  {
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.header.frame_id = planning_frame;
    remove_object.id = object_id;
    remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene->processCollisionObjectMsg(remove_object);
  }
  active_collision_object_ids_.clear();

  // 再把这次调用传入的新障碍物逐个注入 planning scene。
  active_collision_object_ids_.reserve(obstacles.size());
  for (std::size_t idx = 0; idx < obstacles.size(); ++idx)
  {
    const std::string object_id =
      "moveit_obstacle_" + obstacles[idx].name + "_" + std::to_string(idx);
    scene->processCollisionObjectMsg(
      makeSphereCollisionObject(obstacles[idx], planning_frame, object_id));
    active_collision_object_ids_.push_back(object_id);
  }

  // 手动触发一次 scene update，让 MoveIt 规划线程立刻看到最新碰撞环境。
  scene_monitor->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  return true;
}

bool MoveItObstacleAvoidancePlanner::applyStartState(const Eigen::VectorXd & initial_guess)
{
  moveit::core::RobotState start_state(moveit_cpp_->getRobotModel());
  {
    auto scene_monitor = moveit_cpp_->getPlanningSceneMonitorNonConst();
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor);
    // 先复制当前场景状态，未显式指定初值时就以它为规划起点。
    start_state = scene->getCurrentState();
  }

  if (initial_guess.size() == 0)
  {
    planning_component_->setStartState(start_state);
    return true;
  }

  const std::size_t variable_count = joint_model_group_->getVariableCount();
  if (static_cast<std::size_t>(initial_guess.size()) != variable_count)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Initial guess dimension mismatch: expected %zu, got %ld.",
      variable_count,
      initial_guess.size());
    return false;
  }

  std::vector<double> joint_values(variable_count, 0.0);
  for (std::size_t idx = 0; idx < variable_count; ++idx)
  {
    joint_values[idx] = initial_guess(static_cast<Eigen::Index>(idx));
  }

  // `setJointGroupPositions()` 只写入关节组变量，`update()` 再刷新整机派生状态。
  start_state.setJointGroupPositions(joint_model_group_, joint_values);
  start_state.update();
  planning_component_->setStartState(start_state);
  return true;
}

bool MoveItObstacleAvoidancePlanner::extractJointWaypoints(
  std::vector<Eigen::VectorXd> & joint_waypoints) const
{
  joint_waypoints.clear();
  const std::size_t point_count = last_joint_trajectory_.points.size();
  const std::size_t joint_count = joint_model_group_->getVariableCount();
  joint_waypoints.reserve(point_count);

  // 这里只提取位置轨迹点，便于和当前项目其他关节路径接口保持一致。
  for (const trajectory_msgs::msg::JointTrajectoryPoint & point : last_joint_trajectory_.points)
  {
    if (point.positions.size() != joint_count)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "MoveIt returned a trajectory point with %zu positions, expected %zu.",
        point.positions.size(),
        joint_count);
      joint_waypoints.clear();
      return false;
    }

    Eigen::VectorXd q(static_cast<Eigen::Index>(joint_count));
    for (std::size_t idx = 0; idx < joint_count; ++idx)
    {
      // MoveIt 轨迹点里的 `positions` 顺序与 joint model group 的变量顺序一致。
      q(static_cast<Eigen::Index>(idx)) = point.positions[idx];
    }
    joint_waypoints.push_back(std::move(q));
  }

  return !joint_waypoints.empty();
}

std::string MoveItObstacleAvoidancePlanner::resolvePlanningFrame() const
{
  // 用户显式指定参考系时，优先按配置走。
  if (!options_.planning_frame.empty())
  {
    return options_.planning_frame;
  }

  // 否则回退到 robot model 的默认世界坐标系名称。
  if (moveit_cpp_ && moveit_cpp_->getRobotModel())
  {
    return moveit_cpp_->getRobotModel()->getModelFrame();
  }

  // 最后的兜底值仍保持常见的 `world`，避免空字符串传到下游接口。
  return "world";
}
}  // namespace robot_core
