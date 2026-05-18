#include "robot_core/filters.hpp"

/**
 * @file filters.cc
 * @brief 实现控制链路中复用的一维滤波器。
 */

#include <cmath>
#include <stdexcept>

namespace robot_core
{
namespace
{
constexpr double kMinPositiveDt = 1.0e-6;
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

LowPassFilter::LowPassFilter(double cutoff_frequency)
: cutoff_frequency_(cutoff_frequency)
{
  if (cutoff_frequency_ <= 0.0)
  {
    throw std::runtime_error("LowPassFilter cutoff_frequency must be greater than zero.");
  }
}

void LowPassFilter::reset(double initial_value)
{
  last_output_ = initial_value;
  initialized_ = true;
}

double LowPassFilter::filter(double input, double dt)
{
  // 首次进入时直接把输出对齐输入，避免滤波器无历史状态时产生伪过渡。
  if (!initialized_)
  {
    reset(input);
    return last_output_;
  }

  // 采样时间过小时直接退回最小正值，防止离散化系数数值爆炸。
  const double safe_dt = std::max(dt, kMinPositiveDt);
  // 一阶低通的时间常数由截止频率决定。
  const double tau = 1.0 / (kTwoPi * cutoff_frequency_);
  // 标准离散化形式：alpha = dt / (tau + dt)。
  const double alpha = safe_dt / (tau + safe_dt);

  // 当前输出是“上一时刻输出”和“当前输入”的凸组合。
  last_output_ = alpha * input + (1.0 - alpha) * last_output_;
  return last_output_;
}

OneEuroFilter::OneEuroFilter(
  double min_cutoff,
  double beta,
  double derivative_cutoff)
: min_cutoff_(min_cutoff),
  beta_(beta),
  derivative_cutoff_(derivative_cutoff)
{
  if (min_cutoff_ <= 0.0)
  {
    throw std::runtime_error("OneEuroFilter min_cutoff must be greater than zero.");
  }
  if (derivative_cutoff_ <= 0.0)
  {
    throw std::runtime_error("OneEuroFilter derivative_cutoff must be greater than zero.");
  }
}

void OneEuroFilter::reset(double initial_value)
{
  last_input_ = initial_value;
  last_output_ = initial_value;
  // 导数初值置零，表示刚初始化时默认输入变化率未知但近似静止。
  last_filtered_derivative_ = 0.0;
  initialized_ = true;
}

double OneEuroFilter::filter(double input, double dt)
{
  if (!initialized_)
  {
    reset(input);
    return last_output_;
  }

  const double safe_dt = std::max(dt, kMinPositiveDt);
  // 先用原始输入差分估算一阶导数，近似信号变化率。
  const double raw_derivative = (input - last_input_) / safe_dt;
  // 再对导数做一层低通，避免噪声导致自适应截止频率剧烈跳动。
  const double filtered_derivative = filterDerivative(raw_derivative, safe_dt);
  // One Euro 的核心：信号变化越快，主通道截止频率越高。
  const double adaptive_cutoff = min_cutoff_ + beta_ * std::fabs(filtered_derivative);
  const double alpha = computeAlpha(adaptive_cutoff, safe_dt);

  // 主通道仍是一阶低通，只是截止频率随着当前运动速度自动变化。
  last_output_ = alpha * input + (1.0 - alpha) * last_output_;
  last_input_ = input;
  return last_output_;
}

double OneEuroFilter::computeAlpha(double cutoff_frequency, double dt) const
{
  const double safe_cutoff = std::max(cutoff_frequency, 1.0e-6);
  const double safe_dt = std::max(dt, kMinPositiveDt);
  const double tau = 1.0 / (kTwoPi * safe_cutoff);
  return safe_dt / (tau + safe_dt);
}

double OneEuroFilter::filterDerivative(double derivative, double dt)
{
  const double alpha = computeAlpha(derivative_cutoff_, dt);
  last_filtered_derivative_ =
    alpha * derivative + (1.0 - alpha) * last_filtered_derivative_;
  return last_filtered_derivative_;
}

std::unique_ptr<ScalarFilterBase> createScalarFilter(
  const std::string & filter_type,
  double low_pass_cutoff,
  double one_euro_min_cutoff,
  double one_euro_beta,
  double one_euro_derivative_cutoff)
{
  if (filter_type == "low_pass")
  {
    return std::make_unique<LowPassFilter>(low_pass_cutoff);
  }

  if (filter_type == "one_euro")
  {
    return std::make_unique<OneEuroFilter>(
      one_euro_min_cutoff,
      one_euro_beta,
      one_euro_derivative_cutoff);
  }

  throw std::runtime_error(
          "Unsupported filter_type '" + filter_type +
          "'. Supported values are: low_pass, one_euro.");
}
}  // namespace robot_core
