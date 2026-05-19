#include "robot_core/examples/experiment_examples.hpp"

/**
 * @file experiment_examples.cc
 * @brief 实现用于算法对照试验的统一示例逻辑。
 */

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>

namespace robot_core
{
namespace
{
std::string formatJointVector(const Eigen::VectorXd & values)
{
  // 这个辅助函数只负责把 Eigen 向量转成一行可读日志，
  // 便于在实验摘要里直接打印最终关节角和偏差向量。
  std::ostringstream stream;
  stream << "[";
  for (Eigen::Index idx = 0; idx < values.size(); ++idx)
  {
    if (idx > 0)
    {
      stream << ", ";
    }
    stream << values[idx];
  }
  stream << "]";
  return stream.str();
}

Eigen::VectorXd computeNormalizedCenterOffset(const RobotModel & robot_model,
                                              const Eigen::VectorXd & q)
{
  // 这里把每个关节“离上下限中点有多远”归一化到近似 [-1, 1] 的尺度，
  // 这样不同量程的关节就能放在同一个日志里做横向比较。
  const Eigen::VectorXd lower = robot_model.lowerPositionLimits();
  const Eigen::VectorXd upper = robot_model.upperPositionLimits();
  const Eigen::VectorXd center = 0.5 * (lower + upper);
  const Eigen::VectorXd half_range = 0.5 * (upper - lower);

  Eigen::VectorXd normalized_offset = Eigen::VectorXd::Zero(q.size());
  for (Eigen::Index idx = 0; idx < q.size(); ++idx)
  {
    const double safe_half_range = std::max(half_range[idx], 1.0e-6);
    normalized_offset[idx] = (q[idx] - center[idx]) / safe_half_range;
  }

  return normalized_offset;
}

struct PotentialFieldMetrics
{
  /// 整条路径上所有碰撞检测 frame 相对所有障碍物的最小净距离。
  double min_clearance{std::numeric_limits<double>::infinity()};

  /// 排斥势场被激活的次数。
  /// 这里按“路径点 x frame x obstacle”的命中次数累计。
  std::size_t repulsive_activation_count{0U};

  /// 末端最终位置与目标位置之间的欧氏距离。
  double final_position_error{std::numeric_limits<double>::infinity()};
};

PotentialFieldMetrics evaluatePotentialFieldMetrics(
  RobotModel & robot_model,
  const std::vector<Eigen::VectorXd> & joint_waypoints,
  const geometry_msgs::msg::Pose & target_pose,
  const std::vector<std::string> & collision_frames,
  const std::vector<SphericalObstacle> & obstacles,
  double influence_distance)
{
  PotentialFieldMetrics metrics;

  // 若没有障碍物，就不存在“最小净距离”和“排斥激活”的意义；
  // 这里保留 `min_clearance = inf`，日志里就能一眼看出是无障碍场景。
  if (joint_waypoints.empty())
  {
    return metrics;
  }

  for (const Eigen::VectorXd & q : joint_waypoints)
  {
    for (const std::string & frame_name : collision_frames)
    {
      if (!robot_model.hasFrame(frame_name))
      {
        continue;
      }

      pinocchio::SE3 frame_pose;
      if (!robot_model.framePose(q, frame_name, frame_pose))
      {
        continue;
      }

      for (const SphericalObstacle & obstacle : obstacles)
      {
        const double distance_to_center = (frame_pose.translation() - obstacle.center).norm();
        const double protected_radius = obstacle.radius + obstacle.safety_margin;
        const double clearance = distance_to_center - protected_radius;

        metrics.min_clearance = std::min(metrics.min_clearance, clearance);
        if (clearance < influence_distance)
        {
          ++metrics.repulsive_activation_count;
        }
      }
    }
  }

  geometry_msgs::msg::Pose final_pose;
  if (robot_model.forwardKinematics(joint_waypoints.back(), final_pose))
  {
    const Eigen::Vector3d final_position(
      final_pose.position.x, final_pose.position.y, final_pose.position.z);
    const Eigen::Vector3d target_position(
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
    metrics.final_position_error = (final_position - target_position).norm();
  }

  return metrics;
}
}  // namespace

ExperimentExamples::ExperimentExamples(std::shared_ptr<RobotModel> robot_model)
: robot_model_(std::move(robot_model))
{
}

bool ExperimentExamples::runNullspaceIKDemo(const ExperimentExampleInput & input,
                                            const IKOptions & ik_options,
                                            ExperimentExampleOutput & output) const
{
  // 每次实验开始前都重置输出，避免调用方误用上一次结果。
  output = ExperimentExampleOutput{};

  if (!robot_model_ || !robot_model_->isModelLoaded())
  {
    return false;
  }

  // 先把起始关节姿态映射到末端空间，得到这次实验路径的笛卡尔起点。
  geometry_msgs::msg::Pose start_pose;
  if (!robot_model_->forwardKinematics(input.start_configuration, start_pose))
  {
    return false;
  }

  // 这里先在笛卡尔空间构造一条离散路径，
  // 然后让两组 IK 都沿着完全相同的末端目标序列求解，保证对照公平。
  const std::vector<geometry_msgs::msg::Pose> cartesian_waypoints =
    interpolateCartesianWaypoints(
    start_pose, input.target_pose, std::max(2, input.cartesian_waypoint_count));

  // 基线组：显式关闭 joint-limit null-space。
  // 这样得到的是“只做主任务末端跟踪”的纯 IK 结果。
  IKOptions baseline_options = ik_options;
  baseline_options.enable_joint_limit_nullspace = false;
  IKSolver baseline_solver(robot_model_, baseline_options);

  std::vector<Eigen::VectorXd> baseline_joint_waypoints;
  if (!baseline_solver.solveCartesianPath(
      cartesian_waypoints, baseline_joint_waypoints, input.start_configuration))
  {
    return false;
  }

  // 实验组：开启 joint-limit null-space。
  // 主任务仍然是末端跟踪，但在冗余自由度上额外加入“尽量远离关节极限”的次任务。
  IKOptions nullspace_options = ik_options;
  nullspace_options.enable_joint_limit_nullspace = true;
  IKSolver nullspace_solver(robot_model_, nullspace_options);

  if (!nullspace_solver.solveCartesianPath(
      cartesian_waypoints, output.joint_waypoints, input.start_configuration))
  {
    return false;
  }

  // 上层 demo 节点最终要发布的是离散轨迹点，
  // 所以这里把 null-space 实验组得到的关节路径继续插值成可播放轨迹。
  if (!buildTrajectoryFromWaypoints(
      output.joint_waypoints,
      input.segment_duration,
      input.sample_period,
      output.trajectory_points))
  {
    return false;
  }

  // 对照时只比较两组“最终收敛姿态”的关节居中代价。
  // 这样最直接反映 null-space 次任务是否把姿态往中间区域拉回去了。
  const double baseline_cost = computeJointCenteringCost(baseline_joint_waypoints.back());
  const double nullspace_cost = computeJointCenteringCost(output.joint_waypoints.back());
  const Eigen::VectorXd joint_center =
    0.5 * (robot_model_->lowerPositionLimits() + robot_model_->upperPositionLimits());
  const Eigen::VectorXd baseline_offset =
    computeNormalizedCenterOffset(*robot_model_, baseline_joint_waypoints.back());
  const Eigen::VectorXd nullspace_offset =
    computeNormalizedCenterOffset(*robot_model_, output.joint_waypoints.back());

  // 日志摘要除了总代价外，还打印：
  // - 关节上下限中点
  // - 两组最终关节角
  // - 两组相对中心的归一化偏差
  // 便于直接把终端输出拿去做实验记录。
  std::ostringstream summary;
  summary << std::fixed << std::setprecision(6)
          << "Null-space IK demo finished. "
          << "baseline_centering_cost=" << baseline_cost
          << ", nullspace_centering_cost=" << nullspace_cost
          << ", waypoint_count=" << output.joint_waypoints.size()
          << "\njoint_center=" << formatJointVector(joint_center)
          << "\nbaseline_final_q=" << formatJointVector(baseline_joint_waypoints.back())
          << "\nnullspace_final_q=" << formatJointVector(output.joint_waypoints.back())
          << "\nbaseline_center_offset_normed=" << formatJointVector(baseline_offset)
          << "\nnullspace_center_offset_normed=" << formatJointVector(nullspace_offset);
  output.summary = summary.str();
  return true;
}

bool ExperimentExamples::runPotentialFieldAvoidanceDemo(const ExperimentExampleInput & input,
                                                        const ObstacleAvoidanceOptions & options,
                                                        ExperimentExampleOutput & output) const
{
  output = ExperimentExampleOutput{};

  // 势场法实验直接把目标 pose 和障碍物集合交给规划器，
  // 由规划器在关节空间里逐步迭代出一条离散避障路径。
  ObstacleAvoidancePlanner planner(robot_model_, options);
  if (!planner.planToPose(
      input.target_pose, input.obstacles, output.joint_waypoints, input.start_configuration))
  {
    return false;
  }

  // 势场法输出的仍然只是关节路径点，
  // 所以后续和其他实验一样，统一交给轨迹插值器生成可发布轨迹。
  if (!buildTrajectoryFromWaypoints(
      output.joint_waypoints,
      input.segment_duration,
      input.sample_period,
      output.trajectory_points))
  {
    return false;
  }

  // 为了让势场法的避障效果不只停留在“看起来绕开了”，
  // 这里额外统计几项可直接写进实验记录的量化指标。
  PotentialFieldMetrics metrics = evaluatePotentialFieldMetrics(
    *robot_model_,
    output.joint_waypoints,
    input.target_pose,
    options.collision_frames,
    input.obstacles,
    options.influence_distance);

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(6)
          << "Potential-field avoidance demo finished. "
          << "waypoint_count=" << output.joint_waypoints.size()
          << ", obstacle_count=" << input.obstacles.size()
          << "\nmin_clearance=" << metrics.min_clearance
          << "\nrepulsive_activation_count=" << metrics.repulsive_activation_count
          << "\nfinal_position_error=" << metrics.final_position_error;
  output.summary = summary.str();
  return true;
}

bool ExperimentExamples::runMoveItAvoidanceDemo(const rclcpp::Node::SharedPtr & node,
                                                const ExperimentExampleInput & input,
                                                const MoveItObstacleAvoidanceOptions & options,
                                                ExperimentExampleOutput & output) const
{
  output = ExperimentExampleOutput{};

  // MoveIt 实验把环境碰撞和采样规划都交给 MoveIt 2，
  // 这里的职责只是装配输入、接收结果并转成统一输出格式。
  MoveItObstacleAvoidancePlanner planner(node, options);
  if (!planner.planToPose(
      input.target_pose, input.obstacles, output.joint_waypoints, input.start_configuration))
  {
    return false;
  }

  // 即使 MoveIt 已经给出了离散路径点，
  // 仍然统一转成项目内部的 `JointTrajectoryPoint` 序列，便于 demo 节点复用同一发布逻辑。
  if (!buildTrajectoryFromWaypoints(
      output.joint_waypoints,
      input.segment_duration,
      input.sample_period,
      output.trajectory_points))
  {
    return false;
  }

  std::ostringstream summary;
  summary << "MoveIt avoidance demo finished. "
          << "waypoint_count=" << output.joint_waypoints.size()
          << ", obstacle_count=" << input.obstacles.size();
  output.summary = summary.str();
  return true;
}

std::vector<geometry_msgs::msg::Pose> ExperimentExamples::interpolateCartesianWaypoints(
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & target_pose,
  int waypoint_count) const
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  const int clamped_count = std::max(2, waypoint_count);
  waypoints.reserve(static_cast<std::size_t>(clamped_count));

  // 位置插值和姿态插值分开处理：
  // - 位置使用线性插值
  // - 姿态使用四元数球面插值 slerp
  const Eigen::Vector3d start_position(
    start_pose.position.x, start_pose.position.y, start_pose.position.z);
  const Eigen::Vector3d target_position(
    target_pose.position.x, target_pose.position.y, target_pose.position.z);
  const Eigen::Quaterniond start_quaternion(
    start_pose.orientation.w,
    start_pose.orientation.x,
    start_pose.orientation.y,
    start_pose.orientation.z);
  const Eigen::Quaterniond target_quaternion(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z);

  for (int idx = 0; idx < clamped_count; ++idx)
  {
    // `alpha` 描述当前中间点在整条笛卡尔路径上的相对进度。
    const double alpha = static_cast<double>(idx) / static_cast<double>(clamped_count - 1);
    const Eigen::Vector3d position = (1.0 - alpha) * start_position + alpha * target_position;
    const Eigen::Quaterniond quaternion = start_quaternion.slerp(alpha, target_quaternion);

    geometry_msgs::msg::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    waypoints.push_back(pose);
  }

  return waypoints;
}

bool ExperimentExamples::buildTrajectoryFromWaypoints(
  const std::vector<Eigen::VectorXd> & joint_waypoints,
  double segment_duration,
  double sample_period,
  std::vector<JointTrajectoryPoint> & trajectory_points) const
{
  trajectory_points.clear();

  if (joint_waypoints.empty())
  {
    return false;
  }

  // 只有一个路径点时没必要做多段插值，
  // 直接返回一个静态采样点即可。
  if (joint_waypoints.size() == 1)
  {
    JointTrajectoryPoint point;
    point.time_from_start = 0.0;
    point.positions = joint_waypoints.front();
    point.velocities = Eigen::VectorXd::Zero(joint_waypoints.front().size());
    point.accelerations = Eigen::VectorXd::Zero(joint_waypoints.front().size());
    trajectory_points.push_back(std::move(point));
    return true;
  }

  // 多个路径点时，统一交给三次多项式轨迹规划器做平滑离散。
  JointTrajectoryPlanner planner(robot_model_, sample_period);
  return planner.planWaypoints(
    joint_waypoints, segment_duration, trajectory_points, sample_period);
}

double ExperimentExamples::computeJointCenteringCost(const Eigen::VectorXd & q) const
{
  // 当前代价函数不是简单看“某个关节是否为 0”，
  // 而是看整组关节相对各自上下限中点的归一化平方和。
  const Eigen::VectorXd lower = robot_model_->lowerPositionLimits();
  const Eigen::VectorXd upper = robot_model_->upperPositionLimits();
  const Eigen::VectorXd center = 0.5 * (lower + upper);
  const Eigen::VectorXd half_range = 0.5 * (upper - lower);

  double cost = 0.0;
  for (Eigen::Index idx = 0; idx < q.size(); ++idx)
  {
    const double safe_half_range = std::max(half_range[idx], 1.0e-6);
    const double normalized_offset = (q[idx] - center[idx]) / safe_half_range;
    cost += 0.5 * normalized_offset * normalized_offset;
  }

  return cost;
}
}  // namespace robot_core
