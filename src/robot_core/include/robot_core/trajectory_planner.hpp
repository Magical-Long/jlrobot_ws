#pragma once

/**
 * @file trajectory_planner.hpp
 * @brief 定义关节空间轨迹插值与消息转换接口。
 *
 * 该文件专注于“给定离散关节路径点后，如何生成时间参数化的轨迹采样点”，
 * 当前主要提供三次多项式插值与 `JointState` 消息转换能力。
 */

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_core/robot_model.hpp"

namespace robot_core
{
  /**
   * @brief 单个关节轨迹采样点。
   *
   * 该结构体描述某一时刻的关节位置、速度和加速度。
   */
  struct JointTrajectoryPoint
  {
    /// 从总的轨迹起点开始累计的时间，单位为秒。
    /// 上层通常用它来决定“当前应该播放到哪个采样点”。
    double time_from_start{0.0};

    /// 该时刻对应的关节位置。
    /// 维度应与机器人自由度 `dof()` 保持一致。
    Eigen::VectorXd positions;

    /// 该时刻对应的关节速度。
    /// 这里是对 `positions` 的一阶时间导数。
    Eigen::VectorXd velocities;

    /// 该时刻对应的关节加速度。
    /// 这里是对 `positions` 的二阶时间导数。
    Eigen::VectorXd accelerations;
  };

  /**
   * @brief 七轴机械臂关节空间轨迹规划器。
   *
   * 当前提供基于三次多项式的轨迹插值接口：
   * 1. 单段起点到终点插值；
   * 2. 多段路径点串联插值。
   *
   * 这个类目前不做动力学优化，也不做碰撞检测，
   * 它的目标是先提供一个简单、稳定、便于调试的关节轨迹生成器。
   *
   * 因此它很适合：
   * - 验证逆运动学输出是否平滑；
   * - 给 RViz 或仿真器喂一条可视化轨迹；
   * - 为后续更复杂的时间参数化算法提供基线实现。
   */
  class JointTrajectoryPlanner
  {
  public:
    /**
     * @brief 构造轨迹规划器。
     *
     * @param robot_model 已加载的机器人模型对象。
     * @param default_sample_period 默认采样周期，单位为秒。
     */
    explicit JointTrajectoryPlanner(std::shared_ptr<RobotModel> robot_model,
                                    double default_sample_period = 0.02);

    /**
     * @brief 规划单段三次多项式关节轨迹。
     *
     * 该方法默认使用零起始速度和零终止速度，
     * 适合点到点的平滑关节运动。
     *
     * 从数学上说，它会为每个关节分别构造：
     * `q(t) = a0 + a1 t + a2 t^2 + a3 t^3`
     * 再把所有关节在同一采样时刻的结果打包成一个 `JointTrajectoryPoint`。
     *
     * @param start 起始关节角。
     * @param goal 目标关节角。
     * @param duration 轨迹总时长，单位为秒。
     * @param trajectory 输出离散轨迹点序列。
     * @param sample_period 采样周期，单位为秒；若小于等于 0，则使用默认采样周期。
     * @return true 规划成功。
     * @return false 输入维度错误、关节超限或持续时间非法。
     */
    bool planCubic(const Eigen::VectorXd &start,
                   const Eigen::VectorXd &goal,
                   double duration,
                   std::vector<JointTrajectoryPoint> &trajectory,
                   double sample_period = 0.0) const;

    /**
     * @brief 对多个关节路径点进行分段三次插值。
     *
     * 每一段都会调用 `planCubic()` 进行平滑插值，并自动拼接成完整轨迹。
     * 因此这个接口更像“路径离散器 + 时间展开器”，
     * 而不是直接做全局最优轨迹优化。
     *
     * @param waypoints 关节路径点序列，至少包含两个点。
     * @param segment_duration 每一段轨迹的持续时间，单位为秒。
     * @param trajectory 输出离散轨迹点序列。
     * @param sample_period 采样周期，单位为秒；若小于等于 0，则使用默认采样周期。
     * @return true 所有轨迹段都规划成功。
     * @return false 路径点数量不足、维度不匹配或关节越界。
     */
    bool planWaypoints(const std::vector<Eigen::VectorXd> &waypoints,
                       double segment_duration,
                       std::vector<JointTrajectoryPoint> &trajectory,
                       double sample_period = 0.0) const;

    /**
     * @brief 将单个轨迹点转换为 ROS `JointState` 消息。
     *
     * 便于将内部规划结果直接发布到 ROS 通信链路中。
     * 当前实现中：
     * - `position` 对应关节位置；
     * - `velocity` 对应关节速度；
     * - `effort` 暂存关节加速度，便于调试和观测。
     *
     * @param point 输入轨迹点。
     * @return sensor_msgs::msg::JointState 转换后的关节状态消息。
     */
    sensor_msgs::msg::JointState toJointState(const JointTrajectoryPoint &point) const;

  private:
    /**
     * @brief 检查路径点是否合法。
     *
     * 这里不只检查向量长度，
     * 还会复用 `RobotModel` 的关节限位检查逻辑。
     *
     * @param waypoint 待检查的关节配置。
     * @return true 维度正确且满足关节限位。
     * @return false 维度错误、模型未加载或关节越界。
     */
    bool validateWaypoint(const Eigen::VectorXd &waypoint) const;

    /**
     * @brief 解析实际使用的采样周期。
     *
     * 这样做的好处是：上层既可以给每次规划单独传采样时间，
     * 也可以什么都不传，直接复用规划器构造时的默认值。
     *
     * @param sample_period 用户显式传入的采样周期。
     * @return double 当输入大于 0 时返回输入值，否则返回默认采样周期。
     */
    double resolveSamplePeriod(double sample_period) const;

    /// 提供关节维度、限位和关节名称等基础信息。
    std::shared_ptr<RobotModel> robot_model_;

    /// 当调用方未显式指定采样周期时使用的默认值。
    double default_sample_period_;
  };
} // namespace robot_core
