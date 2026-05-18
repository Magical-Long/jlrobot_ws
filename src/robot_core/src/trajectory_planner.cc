#include "robot_core/trajectory_planner.hpp"

/**
 * @file trajectory_planner.cc
 * @brief 实现关节空间三次多项式插值与轨迹采样逻辑。
 *
 * 该实现面向离线轨迹生成场景，重点是稳定、清晰、便于调试，
 * 而不是追求复杂的时间最优或动力学最优效果。
 */

#include <cmath>

namespace robot_core
{
  JointTrajectoryPlanner::JointTrajectoryPlanner(std::shared_ptr<RobotModel> robot_model,
                                                 double default_sample_period)
      : robot_model_(std::move(robot_model)),
        default_sample_period_(default_sample_period)
  {
    // 构造函数本身不做规划；
    // 它只保存模型引用和默认采样周期，供后续多次调用复用。
  }

  bool JointTrajectoryPlanner::planCubic(const Eigen::VectorXd &start,
                                         const Eigen::VectorXd &goal,
                                         double duration,
                                         std::vector<JointTrajectoryPoint> &trajectory,
                                         double sample_period) const
  {
    // 每次规划前都先清空输出，避免调用方误把旧轨迹当作新结果。
    trajectory.clear();

    // 只有当起点、终点都合法，且轨迹时间大于 0 时，三次插值才有意义。
    if (!validateWaypoint(start) || !validateWaypoint(goal) || duration <= 0.0)
    {
      return false;
    }

    // 统一解析这次实际要使用的离散采样时间。
    const double dt = resolveSamplePeriod(sample_period);
    // `delta` 表示每个关节从起点到终点总共要走多少角度。
    const Eigen::VectorXd delta = goal - start;

    // 三次多项式边界条件：
    // q(0) = start, q(T) = goal
    // qdot(0) = 0, qdot(T) = 0
    //
    // 解出后得到：
    // q(t) = a0 + a1 t + a2 t^2 + a3 t^3
    const Eigen::VectorXd a0 = start;
    const Eigen::VectorXd a1 = Eigen::VectorXd::Zero(start.size());
    const Eigen::VectorXd a2 = 3.0 * delta / (duration * duration);
    const Eigen::VectorXd a3 = -2.0 * delta / (duration * duration * duration);

    // 至少保留起点和终点两个采样点。
    // 即使 `duration` 很短，也不能让轨迹退化成“只有一个点”。
    const int steps = std::max(2, static_cast<int>(std::ceil(duration / dt)) + 1);
    trajectory.reserve(static_cast<std::size_t>(steps));

    for (int step = 0; step < steps; ++step)
    {
      // 最后一个点强制落在精确终点时刻，避免步长整除误差导致时间略超或略短。
      const double t = (step == steps - 1) ? duration : step * dt;

      JointTrajectoryPoint point;
      // 每个离散点都带一个相对轨迹起点的累计时间戳。
      point.time_from_start = t;

      // 同一个多项式可以直接求出位置、一阶导数(速度)、二阶导数(加速度)。
      // 这里的位置和速度和加速度都是基于关节角度的数值，不涉及任何坐标变换。
      point.positions = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
      point.velocities = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t;
      point.accelerations = 2.0 * a2 + 6.0 * a3 * t;
      trajectory.push_back(point);
    }

    return true;
  }

  bool JointTrajectoryPlanner::planWaypoints(const std::vector<Eigen::VectorXd> &waypoints,
                                             double segment_duration,
                                             std::vector<JointTrajectoryPoint> &trajectory,
                                             double sample_period) const
  {
    // 输出轨迹由该函数负责完整重建，因此先清空旧结果。
    trajectory.clear();

    // 分段轨迹至少需要两个路径点；
    // 同时每一段的持续时间也必须是正数。
    if (waypoints.size() < 2 || segment_duration <= 0.0)
    {
      return false;
    }

    // 先把这次总规划要使用的采样时间统一算出来。
    const double dt = resolveSamplePeriod(sample_period);
    // `accumulated_time` 用来把每一段“局部从 0 开始”的时间，换成整条轨迹的全局时间。
    double accumulated_time = 0.0;

    // 每相邻两个路径点生成一段轨迹，再把所有轨迹段串接起来。
    // 轨迹数=路径点数-1
    for (std::size_t idx = 0; idx + 1 < waypoints.size(); ++idx)
    {
      // `segment` 临时保存当前这一小段路径插值出来的离散采样点。
      std::vector<JointTrajectoryPoint> segment;
      // 每一段都复用同一个 `planCubic()` 逻辑，
      // 这样分段规划和单段规划在数学行为上保持一致。
      if (!planCubic(waypoints[idx], waypoints[idx + 1], segment_duration, segment, dt))
      {
        // 某一段失败时，整条轨迹直接判失败，避免输出半条不完整轨迹。
        trajectory.clear();
        return false;
      }

      for (std::size_t point_idx = 0; point_idx < segment.size(); ++point_idx)
      {
        if (idx > 0 && point_idx == 0)
        {
          // 跳过后续轨迹段的首点，避免与上一段终点重复。
          continue;
        }

        // 每段轨迹的时间都是从 0 开始的，
        // 所以需要叠加前面所有段的累计时间，才能得到整条轨迹的统一时间轴。
        segment[point_idx].time_from_start += accumulated_time;
        trajectory.push_back(segment[point_idx]);
      }

      // 当前段拼接完成后，把总时间推进一个段长，
      // 下一段的时间轴就能自然接在本段后面。
      accumulated_time += segment_duration;
    }

    return true;
  }

  sensor_msgs::msg::JointState JointTrajectoryPlanner::toJointState(
      const JointTrajectoryPoint &point) const
  {
    sensor_msgs::msg::JointState joint_state;

    // `name/position/velocity/effort` 四个数组必须严格对齐，
    // RViz 和 robot_state_publisher 会按同一索引解释每个关节的数据。
    // 因此这里先复制统一的关节名称，再按相同顺序填充数值数组。
    joint_state.name = robot_model_->jointNames();
    joint_state.position.assign(point.positions.data(), point.positions.data() + point.positions.size());
    joint_state.velocity.assign(point.velocities.data(), point.velocities.data() + point.velocities.size());
    joint_state.effort.assign(point.accelerations.data(),
                              point.accelerations.data() + point.accelerations.size());
    return joint_state;
  }

  bool JointTrajectoryPlanner::validateWaypoint(const Eigen::VectorXd &waypoint) const
  {
    // 轨迹规划器把“模型已加载、维度匹配、关节不越界”统一视为有效路径点。
    // 这样上层在调用规划器前，不必再重复写一遍同样的检查代码。
    return robot_model_ &&
           robot_model_->isModelLoaded() &&
           waypoint.size() == static_cast<Eigen::Index>(robot_model_->dof()) &&
           robot_model_->isConfigurationValid(waypoint);
  }

  double JointTrajectoryPlanner::resolveSamplePeriod(double sample_period) const
  {
    // 显式传入了正数采样时间时，优先使用调用方这次的设定。
    if (sample_period > 0.0)
    {
      return sample_period;
    }

    // 否则退回到规划器初始化时保存的默认采样时间。
    return default_sample_period_;
  }
} // namespace robot_core
