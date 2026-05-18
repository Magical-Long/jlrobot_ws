#pragma once

/**
 * @file torque_controller_base.hpp
 * @brief 定义关节空间力矩控制器的统一抽象接口。
 */

#include <memory>

#include <Eigen/Core>

#include "robot_core/control_reference.hpp"
#include "robot_core/control_state.hpp"
#include "robot_core/robot_model.hpp"

namespace robot_core
{
/**
 * @brief 力矩控制器抽象基类。
 *
 * 设计目标是把“控制律”本身从 ROS2 节点和 Gazebo 桥接逻辑里抽离出来。
 * 上层节点只需要准备：
 * - 当前状态 `ControlState`
 * - 目标参考 `ControlReference`
 *
 * 然后调用统一接口获得输出力矩 `tau` 即可。
 */
class TorqueControllerBase
{
public:
  /**
   * @brief 构造控制器基类。
   *
   * @param robot_model 已加载的机器人模型，用于提供自由度和关节限位等基础信息。
   */
  explicit TorqueControllerBase(std::shared_ptr<RobotModel> robot_model);

  virtual ~TorqueControllerBase() = default;

  /**
   * @brief 根据当前状态和参考计算关节力矩命令。
   *
   * @param state 当前反馈状态。
   * @param reference 当前目标参考。
   * @param torque_command 输出力矩命令向量 `tau`。
   * @return true 计算成功。
   * @return false 输入维度不匹配或模型未加载。
   */
  virtual bool computeTorque(const ControlState & state,
                             const ControlReference & reference,
                             Eigen::VectorXd & torque_command) const = 0;

protected:
  /**
   * @brief 检查输入状态和参考是否与机器人自由度匹配。
   *
   * @param state 当前状态。
   * @param reference 当前目标参考。
   * @return true 输入合法。
   * @return false 输入非法。
   */
  bool validateInput(const ControlState & state,
                     const ControlReference & reference) const;

  /// 控制器共享的机器人模型引用。
  std::shared_ptr<RobotModel> robot_model_;
};
}  // namespace robot_core
