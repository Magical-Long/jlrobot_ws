#pragma once

/**
 * @file control_state.hpp
 * @brief 定义力矩控制器读取的当前关节状态结构。
 */

#include <cstddef>

#include <Eigen/Core>

namespace robot_core
{
/**
 * @brief 关节空间当前状态。
 *
 * 这个结构统一封装闭环控制器每个周期需要读取的反馈量：
 * - 当前关节位置 `q`
 * - 当前关节速度 `dq`
 *
 * 这样做可以把“状态采集”和“控制律计算”从接口层上解耦。
 */
struct ControlState
{
  /// 当前关节位置向量 `q`。
  Eigen::VectorXd positions;

  /// 当前关节速度向量 `dq`。
  Eigen::VectorXd velocities;

  /**
   * @brief 检查状态维度是否与控制自由度匹配。
   *
   * @param dof 期望自由度数量。
   * @return true 位置和速度维度都匹配。
   * @return false 任意一个向量维度不匹配。
   */
  bool isValid(std::size_t dof) const
  {
    return positions.size() == static_cast<Eigen::Index>(dof) &&
           velocities.size() == static_cast<Eigen::Index>(dof);
  }
};
}  // namespace robot_core
