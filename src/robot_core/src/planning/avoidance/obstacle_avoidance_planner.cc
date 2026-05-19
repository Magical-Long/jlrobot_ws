#include "robot_core/planning/avoidance/obstacle_avoidance_planner.hpp"

/**
 * @file obstacle_avoidance_planner.cc
 * @brief 实现基于势场法的机械臂避障规划逻辑。
 *
 * 整体思路可以概括为：
 * 1. 用末端位姿误差构造吸引项，把机械臂往目标拉；
 * 2. 用链路 frame 到球形障碍物的距离构造排斥项，把机械臂从障碍物旁边推开；
 * 3. 用关节居中项减少长期贴近关节极限的情况；
 * 4. 将三项在关节空间叠加后逐步积分，得到离散关节路径。
 */

#include <algorithm>
#include <stdexcept>

#include <Eigen/Geometry>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace robot_core
{
  namespace
  {
    // 位姿误差在 SE(3) 李代数中通常写成 6 维向量：
    // [dx, dy, dz, dwx, dwy, dwz]^T
    constexpr int kPoseDim = 6;

    Eigen::Matrix<double, kPoseDim, 1> computePoseError(const pinocchio::SE3 &current_pose,
                                                        const pinocchio::SE3 &target_pose)
    {
      // `actInv()` 计算“从当前位姿走到目标位姿”的相对变换。
      const pinocchio::SE3 delta_pose = current_pose.actInv(target_pose);
      // 再用 `log6()` 投影到 6 维李代数向量空间，便于后续和 Jacobian 联合使用。
      return pinocchio::log6(delta_pose).toVector();
    }

    pinocchio::SE3 poseToSE3(const geometry_msgs::msg::Pose &pose)
    {
      // ROS Pose 中姿态用四元数表示，Pinocchio SE3 使用旋转矩阵 + 平移向量。
      const Eigen::Quaterniond quaternion(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);
      return pinocchio::SE3(quaternion.normalized().toRotationMatrix(),
                            Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    }

    Eigen::Vector3d computeRepulsiveVelocity(const Eigen::Vector3d &point_position,
                                             const SphericalObstacle &obstacle,
                                             double influence_distance,
                                             double repulsive_gain)
    {
      // `offset` 从障碍物中心指向当前被检测点。
      const Eigen::Vector3d offset = point_position - obstacle.center;
      const double distance_to_center = offset.norm();
      const double protected_radius = obstacle.radius + obstacle.safety_margin;
      // `clearance` 是“点到障碍物保护边界”的净距离。
      const double clearance = distance_to_center - protected_radius;

      if (distance_to_center <= 1.0e-9 || clearance >= influence_distance)
      {
        // 点落在球心附近时方向不可稳定定义；
        // 点已经离障碍物足够远时，也不需要施加排斥速度。
        return Eigen::Vector3d::Zero();
      }

      const Eigen::Vector3d direction = offset / distance_to_center;
      // 过近时对净距离加下界，避免数值发散得太厉害。
      const double clamped_clearance = std::max(clearance, 1.0e-4);
      const double scale =
          repulsive_gain *
          ((1.0 / clamped_clearance) - (1.0 / influence_distance)) /
          (clamped_clearance * clamped_clearance);
      return scale * direction;
    }
  } // namespace

  ObstacleAvoidancePlanner::ObstacleAvoidancePlanner(
      std::shared_ptr<RobotModel> robot_model,
      const ObstacleAvoidanceOptions &options)
      : robot_model_(std::move(robot_model)),
        options_(options)
  {
  }

  bool ObstacleAvoidancePlanner::planToPose(const geometry_msgs::msg::Pose &target_pose,
                                            const std::vector<SphericalObstacle> &obstacles,
                                            std::vector<Eigen::VectorXd> &joint_waypoints,
                                            const Eigen::VectorXd &initial_guess) const
  {
    return planToPose(poseToSE3(target_pose), obstacles, joint_waypoints, initial_guess);
  }

  bool ObstacleAvoidancePlanner::planToPose(const pinocchio::SE3 &target_pose,
                                            const std::vector<SphericalObstacle> &obstacles,
                                            std::vector<Eigen::VectorXd> &joint_waypoints,
                                            const Eigen::VectorXd &initial_guess) const
  {
    if (!robot_model_ || !robot_model_->isModelLoaded())
    {
      throw std::runtime_error("Robot model must be loaded before obstacle avoidance planning.");
    }

    joint_waypoints.clear();
    // 规划从起始配置开始，因此先把起点压入路径中。
    Eigen::VectorXd q = makeInitialConfiguration(initial_guess);
    joint_waypoints.push_back(q);

    for (int step = 0; step < options_.max_steps; ++step)
    {
      Eigen::Matrix<double, kPoseDim, 1> pose_error;
      pinocchio::Data::Matrix6x ee_jacobian;
      if (!computeEndEffectorTask(q, target_pose, pose_error, ee_jacobian))
      {
        joint_waypoints.clear();
        return false;
      }

      if (isGoalReached(pose_error))
      {
        // 当前位姿已满足终止条件时，直接保留已有路径并返回成功。
        return true;
      }

      // 将吸引项、排斥项和关节居中项在线性层面叠加成一个综合步进方向。
      Eigen::VectorXd joint_step =
          computeAttractiveJointStep(pose_error, ee_jacobian) +
          computeRepulsiveJointStep(q, obstacles) +
          computeJointCenteringStep(q);

      const double step_norm = joint_step.norm();
      if (step_norm > options_.max_joint_step_norm && step_norm > 1.0e-12)
      {
        // 控制每一轮最大关节步长，避免势场叠加后某一步突然过猛。
        joint_step *= options_.max_joint_step_norm / step_norm;
      }

      // 再叠加一次基于速度上限的限制，确保单步位移不超过 `velocity_limit * dt`。
      const Eigen::VectorXd velocity_limited_step =
          joint_step.cwiseMin(robot_model_->velocityLimits() * options_.sample_period)
              .cwiseMax(-robot_model_->velocityLimits() * options_.sample_period);

      // 对一般关节流形使用 `integrate()` 比直接 `q + dq` 更稳妥。
      q = pinocchio::integrate(robot_model_->model(), q, velocity_limited_step);
      q = robot_model_->clampToLimits(q);

      if ((q - joint_waypoints.back()).norm() < 1.0e-6)
      {
        // 若一步更新后几乎没有变化，说明当前参数下已经陷入停滞，提前终止循环。
        break;
      }

      joint_waypoints.push_back(q);
    }

    return false;
  }

  const ObstacleAvoidanceOptions &ObstacleAvoidancePlanner::options() const
  {
    return options_;
  }

  void ObstacleAvoidancePlanner::setOptions(const ObstacleAvoidanceOptions &options)
  {
    options_ = options;
  }

  Eigen::VectorXd ObstacleAvoidancePlanner::makeInitialConfiguration(
      const Eigen::VectorXd &initial_guess) const
  {
    if (initial_guess.size() == robot_model_->model().nq)
    {
      // 外部给了合法维度的初值时，优先使用它，便于用户指定起始姿态。
      return robot_model_->clampToLimits(initial_guess);
    }

    // 未提供合法初值时，退回到机械臂中性位姿。
    return robot_model_->neutralConfiguration();
  }

  bool ObstacleAvoidancePlanner::computeEndEffectorTask(
      const Eigen::VectorXd &q,
      const pinocchio::SE3 &target_pose,
      Eigen::Matrix<double, kPoseDim, 1> &pose_error,
      pinocchio::Data::Matrix6x &jacobian) const
  {
    pinocchio::SE3 current_pose;
    if (!robot_model_->forwardKinematics(q, current_pose))
    {
      return false;
    }

    // 当前末端与目标末端之间的 6 维位姿误差。
    pose_error = computePoseError(current_pose, target_pose);
    if (!options_.hold_orientation)
    {
      // 若用户只关心末端位置，就把姿态误差清零，变成纯位置跟踪任务。
      pose_error.tail<3>().setZero();
    }

    if (!robot_model_->frameJacobian(
            q, robot_model_->endEffectorFrame(), jacobian, pinocchio::LOCAL_WORLD_ALIGNED))
    {
      return false;
    }

    return true;
  }

  Eigen::VectorXd ObstacleAvoidancePlanner::computeAttractiveJointStep(
      const Eigen::Matrix<double, kPoseDim, 1> &pose_error,
      const pinocchio::Data::Matrix6x &jacobian) const
  {
    Eigen::Matrix<double, kPoseDim, 1> desired_twist;
    // 用误差乘增益来等效末端速度，形成一个简单的比例控制律。
    // 前 3 维期望线速度由位置误差决定。
    desired_twist.head<3>() = options_.attractive_position_gain * pose_error.head<3>();
    // 后 3 维期望角速度由姿态误差决定。
    desired_twist.tail<3>() = options_.attractive_orientation_gain * pose_error.tail<3>();

    // 阻尼最小二乘把期望末端速度映射成关节增量。
    const Eigen::Matrix<double, kPoseDim, kPoseDim> system =
        jacobian * jacobian.transpose() +
        options_.jacobian_damping * Eigen::Matrix<double, kPoseDim, kPoseDim>::Identity();
    return jacobian.transpose() * system.ldlt().solve(desired_twist) * options_.sample_period;
  }

  Eigen::VectorXd ObstacleAvoidancePlanner::computeRepulsiveJointStep(
      const Eigen::VectorXd &q,
      const std::vector<SphericalObstacle> &obstacles) const
  {
    Eigen::VectorXd repulsive_step = Eigen::VectorXd::Zero(robot_model_->model().nv);

    for (const std::string &frame_name : options_.collision_frames)
    {
      if (!robot_model_->hasFrame(frame_name))
      {
        continue;
      }

      pinocchio::SE3 frame_pose;
      pinocchio::Data::Matrix6x frame_jacobian;
      if (!robot_model_->framePose(q, frame_name, frame_pose) ||
          !robot_model_->frameJacobian(q, frame_name, frame_jacobian,
                                       pinocchio::LOCAL_WORLD_ALIGNED))
      {
        continue;
      }

      // 对避障来说只关心点的空间平移方向，因此只取 Jacobian 的线速度部分。
      const Eigen::MatrixXd linear_jacobian = frame_jacobian.topRows(3);
      for (const SphericalObstacle &obstacle : obstacles)
      {
        const Eigen::Vector3d repulsive_velocity =
            computeRepulsiveVelocity(frame_pose.translation(),
                                     obstacle,
                                     options_.influence_distance,
                                     options_.repulsive_gain);
        // 把工作空间中的排斥速度方向转回关节空间。
        repulsive_step += linear_jacobian.transpose() * repulsive_velocity * options_.sample_period;
      }
    }

    return repulsive_step;
  }

  Eigen::VectorXd ObstacleAvoidancePlanner::computeJointCenteringStep(const Eigen::VectorXd &q) const
  {
    // 上下限中点可视作“更舒适”的参考姿态。
    const Eigen::VectorXd center =
        0.5 * (robot_model_->lowerPositionLimits() + robot_model_->upperPositionLimits());
    return options_.joint_centering_gain * (center - q) * options_.sample_period;
  }

  bool ObstacleAvoidancePlanner::isGoalReached(
      const Eigen::Matrix<double, kPoseDim, 1> &pose_error) const
  {
    // 只有位置和姿态误差都达标时，才认定任务完成。
    return pose_error.head<3>().norm() <= options_.goal_position_tolerance &&
           pose_error.tail<3>().norm() <= options_.goal_orientation_tolerance;
  }
} // namespace robot_core
