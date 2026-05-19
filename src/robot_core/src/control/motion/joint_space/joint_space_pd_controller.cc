#include "robot_core/control/motion/joint_space/joint_space_pd_controller.hpp"

/**
 * @file joint_space_pd_controller.cc
 * @brief 实现最小可用的关节空间 PD 力矩控制器。
 */

#include <algorithm>
#include <stdexcept>

#include "pinocchio/algorithm/rnea.hpp"

namespace robot_core
{
  TorqueControllerBase::TorqueControllerBase(std::shared_ptr<RobotModel> robot_model)
      : robot_model_(std::move(robot_model))
  {
  }

  bool TorqueControllerBase::validateInput(const ControlState &state,
                                           const ControlReference &reference) const
  {
    return robot_model_ &&
           robot_model_->isModelLoaded() &&
           state.isValid(robot_model_->dof()) &&
           reference.isValid(robot_model_->dof());
  }

  JointSpacePDController::JointSpacePDController(std::shared_ptr<RobotModel> robot_model,
                                                 const JointSpacePDOptions &options)
      : TorqueControllerBase(std::move(robot_model)),
        options_(options)
  {
    if (!robot_model_ || !robot_model_->isModelLoaded())
    {
      throw std::runtime_error(
          "Robot model must be loaded before constructing JointSpacePDController.");
    }

    const Eigen::Index dof = static_cast<Eigen::Index>(robot_model_->dof());
    if (options_.kp.size() != dof || options_.kd.size() != dof)
    {
      throw std::runtime_error("PD gains must match the robot degrees of freedom.");
    }

    if (options_.enable_torque_limits && options_.max_torque.size() != dof)
    {
      throw std::runtime_error("Torque limits must match the robot degrees of freedom.");
    }
  }

  bool JointSpacePDController::computeTorque(const ControlState &state,
                                             const ControlReference &reference,
                                             Eigen::VectorXd &torque_command) const
  {
    const Eigen::Index dof = static_cast<Eigen::Index>(robot_model_->dof());
    torque_command = Eigen::VectorXd::Zero(dof);

    if (!validateInput(state, reference))
    {
      return false;
    }

    // 位置误差反映“当前关节离目标角还有多远”。
    const Eigen::VectorXd position_error = reference.positions - state.positions;
    // 速度误差反映“当前关节速度是否偏离目标速度”。
    const Eigen::VectorXd velocity_error = reference.velocities - state.velocities;

    // -------------------- 模式 1：PD 力矩直接输出 --------------------
    // `enable_inverse_dynamics=false` 时，控制律保持为当前最小可用形式：
    //
    //   tau = Kp * (q_des - q) + Kd * (dq_des - dq)
    //
    // 在这个模式下：
    // - `Kp/Kd` 直接输出“关节力矩修正量”
    // - 不显式构造期望关节角加速度 `ddq_cmd`
    // - 不显式使用 `M(q)`、`C(q,dq)` 这些动力学项
    if (!options_.enable_inverse_dynamics)
    {
      torque_command =
          options_.kp.cwiseProduct(position_error) +
          options_.kd.cwiseProduct(velocity_error);
    }
    else
    {
      // -------------------- 模式 2：Inverse Dynamics / Computed Torque --------------------
      // 开启 `enable_inverse_dynamics=true` 后，控制器升级为 inverse dynamics 形式。
      //
      // 第一步：先在“加速度层”构造当前希望系统实现的关节角加速度：
      //
      //   ddq_cmd = ddq_des + Kd * (dq_des - dq) + Kp * (q_des - q)
      //
      // 这里：
      // - `ddq_des`
      //    来自参考输入的前馈关节角加速度
      // - `Kp * (q_des - q) + Kd * (dq_des - dq)`
      //    表示为了更靠近目标而附加的加速度修正
      // - `ddq_cmd`
      //    是当前周期真正拿去做逆动力学求解的目标关节角加速度
      const Eigen::VectorXd acceleration_command =
          reference.accelerations +
          options_.kd.cwiseProduct(velocity_error) +
          options_.kp.cwiseProduct(position_error);

      const pinocchio::Model &model = robot_model_->model();
      pinocchio::Data &data = robot_model_->data();

      // 第二步：用 Pinocchio 的 `rnea()` 直接求逆动力学力矩。
      // 标准机械臂逆动力学公式为：
      //
      //   tau = M(q) * ddq + C(q, dq) * dq + g(q)
      //
      // 各项具体意义：
      // - `M(q) * ddq`
      //    在当前构型 `q` 下，为了产生目标关节角加速度 `ddq`
      //    需要克服各关节等效惯性所需的力矩
      // - `C(q, dq) * dq`
      //    当前关节角速度 `dq` 存在时，由科氏力 / 离心力带来的速度相关项
      // - `g(q)`
      //    当前姿态下抵消重力所需的关节力矩
      //
      // 因此这里的：
      //
      //   rnea(model, data, q, dq, ddq_cmd)
      //
      // 表示：
      // 在“当前关节角 `q` + 当前关节速度 `dq`”这个真实状态下，
      // 若希望系统接下来产生 `ddq_cmd` 这样的关节角加速度，
      // 那么理论上应该施加的关节力矩是多少。
      //
      // 需要注意：这条逆动力学结果里已经天然包含了重力项 `g(q)`，
      // 因此在 inverse dynamics 模式下，不应再额外手工叠加一次重力补偿。
      torque_command =
          pinocchio::rnea(
              model,
              data,
              state.positions,
              state.velocities,
              acceleration_command);
    }

    // -------------------- 仅在 PD 模式下补单独重力项 --------------------
    // 若未启用完整逆动力学，但启用了重力补偿，则基于当前真实关节角 `q`
    // 计算“仅为了托住机械臂抗重力”所需的关节力矩 `g(q)`。
    //
    // 这里直接复用 Pinocchio 的逆动力学 `rnea()`。
    // 对机械臂的标准逆动力学，关节力矩公式可写成：
    //
    //   tau = M(q) * ddq + C(q, dq) * dq + g(q)
    //
    // 其中各项的物理意义如下：
    // - `q`
    //    当前关节角 / 当前机器人构型。
    // - `dq`
    //    当前关节角速度。
    // - `ddq`
    //    期望关节角加速度。
    // - `M(q) * ddq`
    //    为了产生目标关节加速度，需要克服当前构型下等效惯性所需的力矩。
    // - `C(q, dq) * dq`
    //    当前运动状态下的科氏力 / 离心力等速度相关项。
    // - `g(q)`
    //    当前姿态下抵消重力所需的关节力矩。
    //
    // 而当前这个控制器还不是完整的 computed torque / inverse dynamics 控制，
    // 这里只是借用 `rnea()` 的一个“退化情形”来提取重力项：
    //
    //   dq  = 0
    //   ddq = 0
    //
    // 此时逆动力学公式退化为：
    //
    //   tau = g(q)
    //
    // 因而 `rnea(model, data, q, 0, 0)` 返回的就是当前构型下的重力补偿力矩。
    //
    // 再次强调：这一步只属于 “PD + g(q)” 模式；
    // 若已经启用完整 inverse dynamics，上面那条 `rnea(q, dq, ddq_cmd)` 已经包含重力项，
    // 这里就不能再重复相加。
    if (!options_.enable_inverse_dynamics && options_.enable_gravity_compensation)
    {
      const pinocchio::Model &model = robot_model_->model();
      pinocchio::Data &data = robot_model_->data();

      const Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(model.nv);
      const Eigen::VectorXd zero_acceleration = Eigen::VectorXd::Zero(model.nv);
      const Eigen::VectorXd gravity_torque =
          pinocchio::rnea(
              model,
              data,
              state.positions,
              zero_velocity,
              zero_acceleration);

      // 这样控制律从“纯 PD”升级为：
      // tau = Kp (q_des - q) + Kd (dq_des - dq) + g(q)
      //
      // 重力项负责先把机械臂托住，PD 项再去修正剩余跟踪误差。
      torque_command += gravity_torque;
    }

    if (options_.enable_torque_limits)
    {
      for (Eigen::Index idx = 0; idx < torque_command.size(); ++idx)
      {
        const double limit = std::max(0.0, options_.max_torque[idx]);
        torque_command[idx] = std::max(-limit, std::min(limit, torque_command[idx]));
      }
    }

    return true;
  }

  const JointSpacePDOptions &JointSpacePDController::options() const
  {
    return options_;
  }
} // namespace robot_core
