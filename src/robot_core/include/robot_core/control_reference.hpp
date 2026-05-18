#pragma once

/**
 * @file control_reference.hpp
 * @brief 定义力矩控制器消费的目标参考结构。
 */

#include <cstddef>

#include <Eigen/Core>

namespace robot_core
{
/**
 * @brief 关节空间控制参考。
 *
 * 这个结构表示控制器当前希望跟踪的目标：
 * - 目标关节位置 `q_des`
 * - 目标关节速度 `dq_des`
 * - 目标关节加速度 `ddq_des`
 *
 * 对最小 PD 控制器而言，只会直接使用位置和速度；
 * 但预留加速度接口后，未来扩展 computed torque 会更顺滑。
 */
struct ControlReference
{
  /// 目标关节位置 `q_des`。
  Eigen::VectorXd positions;

  /// 目标关节速度 `dq_des`。
  Eigen::VectorXd velocities;

  /// 目标关节加速度 `ddq_des`。
  Eigen::VectorXd accelerations;

  /**
   * @brief 检查参考维度是否与控制自由度匹配。
   *
   * @param dof 期望自由度数量。
   * @return true 三个向量维度都匹配。
   * @return false 任意一个向量维度不匹配。
   */
  bool isValid(std::size_t dof) const
  {
    return positions.size() == static_cast<Eigen::Index>(dof) &&
           velocities.size() == static_cast<Eigen::Index>(dof) &&
           accelerations.size() == static_cast<Eigen::Index>(dof);
  }
};
}  // namespace robot_core
