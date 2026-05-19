#pragma once

/**
 * @file filters.hpp
 * @brief 定义控制链路中可复用的标量滤波器接口与常用实现。
 */

#include <memory>
#include <string>

namespace robot_core
{
/**
 * @brief 标量信号滤波器统一抽象接口。
 *
 * 当前主要面向关节速度、力矩等一维时序信号。
 * 之所以先抽象成标量接口，是因为机械臂每个关节都可以各自持有一套滤波器实例，
 * 这样既容易在 ROS2 节点里按关节复用，也方便后续扩展成向量封装。
 */
class ScalarFilterBase
{
public:
  virtual ~ScalarFilterBase() = default;

  /**
   * @brief 用一个已知值重置滤波器内部状态。
   *
   * @param initial_value 作为滤波器新起点的原始输入值。
   */
  virtual void reset(double initial_value) = 0;

  /**
   * @brief 对当前输入值执行一次滤波更新。
   *
   * @param input 当前时刻输入。
   * @param dt 与上一次更新之间的采样时间，单位秒。
   * @return double 当前时刻滤波输出。
   */
  virtual double filter(double input, double dt) = 0;
};

/**
 * @brief 一阶低通滤波器。
 *
 * 它适合先作为“最保守、最容易理解”的滤波基线：
 * - cutoff 低时更平滑，但相位滞后更明显；
 * - cutoff 高时响应更快，但抑制高频噪声更弱。
 */
class LowPassFilter : public ScalarFilterBase
{
public:
  /**
   * @brief 构造一阶低通滤波器。
   *
   * @param cutoff_frequency 截止频率，单位 Hz。
   */
  explicit LowPassFilter(double cutoff_frequency);

  void reset(double initial_value) override;

  double filter(double input, double dt) override;

private:
  /// 截止频率，单位 Hz。
  double cutoff_frequency_;

  /// 上一时刻滤波输出。
  double last_output_{0.0};

  /// 是否已经有过有效历史状态。
  bool initialized_{false};
};

/**
 * @brief One Euro Filter。
 *
 * 这是一个“速度越快、截止频率越高”的自适应低通滤波器。
 * 它很适合关节速度这种场景：
 * - 静止或小幅抖动时，强力抑制高频噪声；
 * - 快速运动时，自动减少滤波滞后。
 */
class OneEuroFilter : public ScalarFilterBase
{
public:
  /**
   * @brief 构造 One Euro Filter。
   *
   * @param min_cutoff 基础截止频率，单位 Hz。
   * @param beta 输入变化率增益，越大表示运动变快时越愿意放宽滤波。
   * @param derivative_cutoff 输入导数内部低通的截止频率，单位 Hz。
   */
  OneEuroFilter(double min_cutoff, double beta, double derivative_cutoff);

  void reset(double initial_value) override;

  double filter(double input, double dt) override;

private:
  /// 计算指定截止频率下的一阶低通平滑系数。
  double computeAlpha(double cutoff_frequency, double dt) const;

  /// 对输入导数做低通滤波，避免自适应截止频率本身被噪声放大。
  double filterDerivative(double derivative, double dt);

  /// 基础截止频率，单位 Hz。
  double min_cutoff_;

  /// 运动速度相关增益。
  double beta_;

  /// 导数低通截止频率，单位 Hz。
  double derivative_cutoff_;

  /// 上一时刻原始输入。
  double last_input_{0.0};

  /// 上一时刻主滤波输出。
  double last_output_{0.0};

  /// 上一时刻导数低通输出。
  double last_filtered_derivative_{0.0};

  /// 是否已经完成首次初始化。
  bool initialized_{false};
};

/**
 * @brief 根据字符串类型创建滤波器实例。
 *
 * @param filter_type 支持 `low_pass` 与 `one_euro`。
 * @param low_pass_cutoff 一阶低通截止频率。
 * @param one_euro_min_cutoff One Euro 最小截止频率。
 * @param one_euro_beta One Euro 自适应增益。
 * @param one_euro_derivative_cutoff One Euro 导数低通截止频率。
 * @return std::unique_ptr<ScalarFilterBase> 创建出的滤波器对象。
 */
std::unique_ptr<ScalarFilterBase> createScalarFilter(
  const std::string & filter_type,
  double low_pass_cutoff,
  double one_euro_min_cutoff,
  double one_euro_beta,
  double one_euro_derivative_cutoff);
}  // namespace robot_core
