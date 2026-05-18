#pragma once

/**
 * @file joint_space_pd_controller.hpp
 * @brief 定义最小可用的关节空间 PD 力矩控制器。
 */

#include <memory>

#include <Eigen/Core>

#include "robot_core/torque_controller_base.hpp"

namespace robot_core
{
/**
 * @brief 关节空间 PD 控制器参数。
 */
struct JointSpacePDOptions
{
  /// 位置环增益 `Kp`。维度应与机械臂自由度一致。
  Eigen::VectorXd kp;

  /// 速度环增益 `Kd`。维度应与机械臂自由度一致。
  Eigen::VectorXd kd;

  /// 是否对输出力矩做幅值限制。
  bool enable_torque_limits{false};

  /// 每个关节允许的最大绝对力矩。
  Eigen::VectorXd max_torque;

  /// 是否在 PD 力矩之外叠加模型重力补偿项 `g(q)`。
  bool enable_gravity_compensation{false};

  /// 是否启用完整的逆动力学 / computed torque 控制。
  /// 开启后控制律不再是“PD 力矩直接输出”，而是：
  /// 1. 先由误差构造期望关节加速度 `ddq_cmd`
  /// 2. 再通过 `tau = M(q)ddq_cmd + C(q,dq)dq + g(q)` 求最终力矩
  bool enable_inverse_dynamics{false};
};

/**
 * @brief 最小可用的关节空间 PD 力矩控制器。
 *
 * 控制律形式：
 *
 * `tau = Kp * (q_des - q) + Kd * (dq_des - dq)`
 *
 * 其中：
 * - `q` / `dq` 来自当前反馈状态；
 * - `q_des` / `dq_des` 来自目标参考；
 * - `tau` 是最终要发送给 effort controller 的关节力矩命令。
 *
 * 这是后续实现重力补偿、computed torque、阻抗控制前最适合的第一步。
 */
class JointSpacePDController : public TorqueControllerBase
{
public:
  /**
   * @brief 构造关节空间 PD 控制器。
   *
   * @param robot_model 已加载的机器人模型。
   * @param options PD 控制参数。
   */
  JointSpacePDController(std::shared_ptr<RobotModel> robot_model,
                         const JointSpacePDOptions & options);

  bool computeTorque(const ControlState & state,
                     const ControlReference & reference,
                     Eigen::VectorXd & torque_command) const override;

  /// 获取当前 PD 参数。
  const JointSpacePDOptions & options() const;

private:
  /// 当前 PD 参数集合。
  JointSpacePDOptions options_;
};
}  // namespace robot_core
