#pragma once

/**
 * @file obstacle_avoidance_planner.hpp
 * @brief 定义基于势场法的机械臂工作空间避障规划器接口。
 *
 * 该文件聚焦“如何在已知目标末端位姿和球形障碍物集合的条件下，
 * 逐步生成一条避障关节路径”。它本身不负责轨迹插值和 ROS 发布，
 * 而是输出一串离散的关节路径点，供上层继续处理。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "robot_core/model/robot_model.hpp"

namespace robot_core
{
/**
 * @brief 工作空间中的球形障碍物描述。
 *
 * 当前实现使用球体近似障碍物，优点是：
 * 1. 距离计算简单；
 * 2. 梯度方向明确；
 * 3. 参数直观，便于在 YAML / launch 里配置。
 */
struct SphericalObstacle
{
  /// 仅用于日志和调试的障碍物名称。
  std::string name{"obstacle"};

  /// 障碍物球心在世界坐标系下的位置，单位为米。
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};

  /// 障碍物物理半径，单位为米。
  double radius{0.05};

  /// 额外安全边界，单位为米。
  double safety_margin{0.03};
};

/**
 * @brief 势场避障规划参数集合。
 *
 * 该规划器使用“末端吸引项 + 链路排斥项 + 关节居中项”的组合：
 * - 吸引项负责把末端往目标位姿拉；
 * - 排斥项负责把参与碰撞检测的 link frame 从障碍物附近推开；
 * - 关节居中项负责减少长期贴近关节极限的情况。
 */
struct ObstacleAvoidanceOptions
{
  /// 最大离散规划步数。
  int max_steps{400};

  /// 每一步的积分时间，单位为秒。
  double sample_period{0.05};

  /// 末端位置吸引增益。
  double attractive_position_gain{1.2};

  /// 末端姿态吸引增益。
  double attractive_orientation_gain{0.8};

  /// 障碍物排斥增益。
  double repulsive_gain{0.015};

  /// 排斥势场生效距离，单位为米。
  double influence_distance{0.20};

  /// 关节居中增益，值越大越倾向回到关节中间区域。
  double joint_centering_gain{0.02};

  /// 数值求解末端速度映射时的阻尼系数。
  double jacobian_damping{1.0e-4};

  /// 终止条件中的位置误差容差，单位为米。
  double goal_position_tolerance{0.01};

  /// 终止条件中的姿态误差容差，单位为弧度。
  double goal_orientation_tolerance{0.08};

  /// 每一步允许的最大关节增量范数。
  double max_joint_step_norm{0.08};

  /// 是否要求在规划过程中同时跟踪末端姿态。
  bool hold_orientation{true};

  /// 用于做链路避障的 frame 名称列表。
  std::vector<std::string> collision_frames{
    "iiwa_link_1",
    "iiwa_link_2",
    "iiwa_link_3",
    "iiwa_link_4",
    "iiwa_link_5",
    "iiwa_link_6",
    "iiwa_link_7",
    "iiwa_link_ee"};
};

/**
 * @brief 面向工作空间球形障碍物的机械臂避障规划器。
 *
 * 这个类不直接输出控制指令，而是离线生成一串关节路径点。
 * 上层既可以把这串路径点交给已有的 `JointTrajectoryPlanner` 做平滑插值，
 * 也可以逐点发送给自己的控制器。
 *
 * 当前算法特点：
 * 1. 使用 Pinocchio 正运动学和 Jacobian；
 * 2. 末端吸引项通过阻尼最小二乘映射到关节空间；
 * 3. 障碍物排斥项通过每个链路 frame 的平移 Jacobian 反推到关节空间；
 * 4. 每一步都使用 `pinocchio::integrate()` 和关节限幅，保持接口通用性。
 */
class ObstacleAvoidancePlanner
{
public:
  /**
   * @brief 构造避障规划器。
   *
   * @param robot_model 已加载的机器人模型对象。
   * @param options 势场规划参数。
   */
  explicit ObstacleAvoidancePlanner(std::shared_ptr<RobotModel> robot_model,
                                    const ObstacleAvoidanceOptions & options =
                                      ObstacleAvoidanceOptions{});

  /**
   * @brief 规划到 ROS Pose 目标位姿的避障关节路径。
   *
   * @param target_pose 目标末端位姿。
   * @param obstacles 工作空间障碍物列表。
   * @param joint_waypoints 输出关节路径点，首点为起始配置。
   * @param initial_guess 起始关节配置；若为空则使用中性位姿。
   * @return true 在给定步数内到达目标附近。
   * @return false 未收敛或输入非法。
   */
  bool planToPose(const geometry_msgs::msg::Pose & target_pose,
                  const std::vector<SphericalObstacle> & obstacles,
                  std::vector<Eigen::VectorXd> & joint_waypoints,
                  const Eigen::VectorXd & initial_guess = Eigen::VectorXd()) const;

  /**
   * @brief 规划到 Pinocchio SE3 目标位姿的避障关节路径。
   *
   * @param target_pose 目标末端位姿。
   * @param obstacles 工作空间障碍物列表。
   * @param joint_waypoints 输出关节路径点，首点为起始配置。
   * @param initial_guess 起始关节配置；若为空则使用中性位姿。
   * @return true 在给定步数内到达目标附近。
   * @return false 未收敛或输入非法。
   */
  bool planToPose(const pinocchio::SE3 & target_pose,
                  const std::vector<SphericalObstacle> & obstacles,
                  std::vector<Eigen::VectorXd> & joint_waypoints,
                  const Eigen::VectorXd & initial_guess = Eigen::VectorXd()) const;

  /**
   * @brief 获取当前规划参数。
   *
   * @return const ObstacleAvoidanceOptions& 当前参数。
   */
  const ObstacleAvoidanceOptions & options() const;

  /**
   * @brief 替换当前规划参数。
   *
   * @param options 新参数。
   */
  void setOptions(const ObstacleAvoidanceOptions & options);

private:
  /**
   * @brief 生成规划起始关节配置。
   *
   * 如果调用方提供了维度正确的初值，就在限幅后直接使用；
   * 否则退回到机器人模型的中性位姿。
   *
   * @param initial_guess 外部传入的起始配置候选值。
   * @return Eigen::VectorXd 最终用于规划的起始关节配置。
   */
  Eigen::VectorXd makeInitialConfiguration(const Eigen::VectorXd & initial_guess) const;

  /**
   * @brief 计算末端吸引任务所需的位姿误差与 Jacobian。
   *
   * 该函数会同时完成：
   * 1. 当前末端位姿的正运动学计算；
   * 2. 当前位姿到目标位姿的 6 维李代数误差计算；
   * 3. 末端 frame 在 `LOCAL_WORLD_ALIGNED` 下的 Jacobian 计算。
   *
   * @param q 当前关节配置。
   * @param target_pose 目标末端位姿。
   * @param pose_error 输出 6 维位姿误差，前 3 维为位置误差，后 3 维为姿态误差。
   * @param jacobian 输出末端 Jacobian，维度为 `6 x model.nv`。
   * @return true 计算成功。
   * @return false 模型状态非法或正运动学/Jacobian 计算失败。
   */
  bool computeEndEffectorTask(const Eigen::VectorXd & q,
                              const pinocchio::SE3 & target_pose,
                              Eigen::Matrix<double, 6, 1> & pose_error,
                              pinocchio::Data::Matrix6x & jacobian) const;

  /**
   * @brief 将末端吸引任务映射成关节空间增量。
   *
   * 这里使用阻尼最小二乘，把笛卡尔空间中的“期望末端 twist”
   * 反解成一个较平滑、较稳定的关节步进方向。
   *
   * @param pose_error 当前末端位姿误差。
   * @param jacobian 当前末端 Jacobian。
   * @return Eigen::VectorXd 由吸引项产生的关节步进。
   */
  Eigen::VectorXd computeAttractiveJointStep(
    const Eigen::Matrix<double, 6, 1> & pose_error,
    const pinocchio::Data::Matrix6x & jacobian) const;

  /**
   * @brief 计算障碍物排斥项对应的关节空间增量。
   *
   * 该函数会遍历配置中的所有碰撞检测 frame，并针对每个球形障碍物：
   * 1. 先在工作空间中计算“远离障碍物”的线速度方向；
   * 2. 再通过对应 frame 的平移 Jacobian 反推到关节空间。
   *
   * @param q 当前关节配置。
   * @param obstacles 工作空间障碍物集合。
   * @return Eigen::VectorXd 由排斥势场产生的关节步进。
   */
  Eigen::VectorXd computeRepulsiveJointStep(const Eigen::VectorXd & q,
                                            const std::vector<SphericalObstacle> & obstacles) const;

  /**
   * @brief 计算关节居中项对应的关节空间增量。
   *
   * 该项会把关节配置往“上下限中点”轻微拉回，作用是：
   * 1. 减少长期贴近关节极限的情况；
   * 2. 给冗余机械臂提供一点更稳定的姿态偏好。
   *
   * @param q 当前关节配置。
   * @return Eigen::VectorXd 由关节居中项产生的关节步进。
   */
  Eigen::VectorXd computeJointCenteringStep(const Eigen::VectorXd & q) const;

  /**
   * @brief 判断当前末端误差是否已经满足停止条件。
   *
   * @param pose_error 当前 6 维位姿误差。
   * @return true 位置和姿态误差都已落入容差范围内。
   * @return false 仍需继续迭代。
   */
  bool isGoalReached(const Eigen::Matrix<double, 6, 1> & pose_error) const;

  /// 机器人模型入口，负责提供正运动学、Jacobian、关节限位等基础能力。
  std::shared_ptr<RobotModel> robot_model_;

  /// 当前避障规划所使用的全部参数配置。
  ObstacleAvoidanceOptions options_;
};
}  // namespace robot_core
