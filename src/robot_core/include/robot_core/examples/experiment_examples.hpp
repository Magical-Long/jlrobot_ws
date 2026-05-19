#pragma once

/**
 * @file experiment_examples.hpp
 * @brief 定义用于算法对照试验的统一示例接口。
 *
 * 这个文件的目标不是替代现有规划器或求解器，
 * 而是把“如何组织一个可复现的实验”这部分公共逻辑收拢起来，
 * 方便后续继续添加更多 demo，例如：
 * 1. null-space / joint-limit avoidance 对照；
 * 2. 势场法避障轨迹规划；
 * 3. MoveIt 避障轨迹规划；
 * 4. 未来的优化型控制或其他 IK/规划算法对照。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_core/planning/avoidance/moveit_obstacle_avoidance_planner.hpp"
#include "robot_core/planning/avoidance/obstacle_avoidance_planner.hpp"
#include "robot_core/planning/ik/ik_solver.hpp"
#include "robot_core/planning/trajectory/trajectory_planner.hpp"

namespace robot_core
{
/**
 * @brief 单次实验所需的公共输入。
 *
 * 这部分信息对多种实验模式都通用：
 * - 目标末端位姿；
 * - 起始关节配置；
 * - 障碍物集合；
 * - 轨迹离散化相关参数。
 */
struct ExperimentExampleInput
{
  /// 末端目标位姿。
  geometry_msgs::msg::Pose target_pose;

  /// 起始关节配置。
  Eigen::VectorXd start_configuration;

  /// 工作空间球形障碍物集合。
  std::vector<SphericalObstacle> obstacles;

  /// 轨迹插值每一段的持续时间，单位为秒。
  double segment_duration{0.15};

  /// 轨迹离散采样周期，单位为秒。
  double sample_period{0.05};

  /// null-space IK demo 中使用的笛卡尔中间路径点数量。
  int cartesian_waypoint_count{8};
};

/**
 * @brief 单次实验的统一输出。
 *
 * 这样无论底层用的是 IK、势场还是 MoveIt，
 * 上层 demo 节点都可以按相同方式读取关节路径和离散轨迹。
 */
struct ExperimentExampleOutput
{
  /// 原始关节路径点，便于算法层面对照。
  std::vector<Eigen::VectorXd> joint_waypoints;

  /// 用于发布和可视化的离散轨迹点。
  std::vector<JointTrajectoryPoint> trajectory_points;

  /// 对本次实验结果的简短摘要，适合直接打印日志。
  std::string summary;
};

/**
 * @brief 算法示例集合。
 *
 * 这个类只负责“组织实验”和“生成可对照结果”，
 * 不直接创建 ROS2 定时器或发布消息。
 */
class ExperimentExamples
{
public:
  /**
   * @brief 构造实验集合对象。
   *
   * @param robot_model 已加载的机器人模型对象。
   */
  explicit ExperimentExamples(std::shared_ptr<RobotModel> robot_model);

  /**
   * @brief 运行 null-space IK 对照示例。
   *
   * 该示例会：
   * 1. 先构造一条从起始末端位姿到目标位姿的笛卡尔路径；
   * 2. 用“关闭 null-space”和“开启 null-space”两种 IK 方式分别求解；
   * 3. 记录两种末端解的关节居中代价，并输出开启 null-space 的轨迹结果。
   *
   * @param input 实验公共输入。
   * @param ik_options IK 参数。
   * @param output 输出关节路径、离散轨迹和摘要信息。
   * @return true 实验成功生成轨迹。
   * @return false IK 求解或轨迹插值失败。
   */
  bool runNullspaceIKDemo(const ExperimentExampleInput & input,
                          const IKOptions & ik_options,
                          ExperimentExampleOutput & output) const;

  /**
   * @brief 运行基于势场法的避障轨迹规划示例。
   *
   * @param input 实验公共输入。
   * @param options 势场法规划参数。
   * @param output 输出关节路径、离散轨迹和摘要信息。
   * @return true 规划成功生成轨迹。
   * @return false 势场规划失败或轨迹插值失败。
   */
  bool runPotentialFieldAvoidanceDemo(const ExperimentExampleInput & input,
                                      const ObstacleAvoidanceOptions & options,
                                      ExperimentExampleOutput & output) const;

  /**
   * @brief 运行基于 MoveIt 的避障轨迹规划示例。
   *
   * @param node ROS2 节点，用于创建 MoveItCpp。
   * @param input 实验公共输入。
   * @param options MoveIt 规划参数。
   * @param output 输出关节路径、离散轨迹和摘要信息。
   * @return true 规划成功生成轨迹。
   * @return false MoveIt 初始化失败、规划失败或轨迹插值失败。
   */
  bool runMoveItAvoidanceDemo(const rclcpp::Node::SharedPtr & node,
                              const ExperimentExampleInput & input,
                              const MoveItObstacleAvoidanceOptions & options,
                              ExperimentExampleOutput & output) const;

private:
  /// 构造一条从起始位姿到目标位姿的简单笛卡尔插值路径。
  std::vector<geometry_msgs::msg::Pose> interpolateCartesianWaypoints(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & target_pose,
    int waypoint_count) const;

  /// 将关节路径点进一步插值成便于发布的离散轨迹。
  bool buildTrajectoryFromWaypoints(const std::vector<Eigen::VectorXd> & joint_waypoints,
                                    double segment_duration,
                                    double sample_period,
                                    std::vector<JointTrajectoryPoint> & trajectory_points) const;

  /// 计算“离关节中心有多远”的代价，用于对照 null-space 效果。
  double computeJointCenteringCost(const Eigen::VectorXd & q) const;

  /// 机器人模型入口，供各类实验共享。
  std::shared_ptr<RobotModel> robot_model_;
};
}  // namespace robot_core
