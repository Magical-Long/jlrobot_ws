#pragma once

/**
 * @file robot_model.hpp
 * @brief 定义对 Pinocchio 机器人模型的统一封装接口。
 *
 * 该文件负责把 URDF、Pinocchio Model/Data 与上层业务代码连接起来，
 * 为 IK、轨迹规划、避障规划等模块提供统一的模型访问入口。
 */

#include <atomic>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace robot_core
{
/**
 * @brief Pinocchio 机器人模型封装类。
 *
 * 这个类负责：
 * 1. 从 URDF 加载七轴机械臂模型；
 * 2. 缓存关节名称、关节位置限位和速度限位；
 * 3. 提供末端执行器的正运动学计算接口；
 * 4. 为逆运动学和轨迹规划提供统一的模型访问入口。
 *
 * 设计上它相当于一个“只加载一次、后续反复查询”的模型容器：
 * - `model_` 保存机器人拓扑结构、关节定义、限位和 frame 信息；
 * - `data_` 是算法运行时复用的缓存区，正运动学/雅可比等算法都会往里面写中间结果；
 * - 其余成员是对上层更友好的封装，避免调用方频繁直接接触 Pinocchio 底层数组。
 *
 * 对当前项目的 iiwa7 来说：
 * - 机械臂有 7 个活动转动关节，因此 `nq == 7`、`nv == 7`；
 * - URDF 中还定义了一个固定末端关节 `iiwa_joint_ee`，它不会在 `q/v` 中占自由度；
 * - Pinocchio 内部额外维护一个 `universe` 根关节，其 joint id 通常为 0。
 */
class RobotModel
{
public:
  /**
   * @brief 构造机器人模型对象。
   *
   * @param urdf_path 机器人 URDF 文件路径。
   * @param end_effector_frame 末端执行器 frame 名称，默认使用 `iiwa_link_ee`。
   */
  RobotModel(const std::string & urdf_path,
             const std::string & end_effector_frame = "iiwa_link_ee");

  /**
   * @brief 从 URDF 文件中加载 Pinocchio 模型。
   *
   * 成功后会自动刷新关节名称、关节限位和末端 frame 索引。
   *
   * @return true 加载成功。
   * @return false 加载失败，例如 URDF 路径错误或末端 frame 不存在。
   */
  bool loadURDF();

  /**
   * @brief 判断模型是否已经成功加载。
   *
   * @return true 模型已加载完成。
   * @return false 模型尚未加载或加载失败。
   */
  bool isModelLoaded() const;

  /**
   * @brief 获取机器人自由度数量。
   *
   * 对当前 iiwa7 机械臂来说，正常情况下应返回 7。
   * 这里返回的是配置空间维度 `nq`，
   * 它在固定基座机械臂里通常等于“可动关节数量”。
   *
   * @return std::size_t 关节配置向量维度。
   */
  std::size_t dof() const;

  /**
   * @brief 获取末端执行器 frame 名称。
   *
   * @return const std::string& 当前使用的末端 frame 名称。
   */
  const std::string & endEffectorFrame() const;

  /**
   * @brief 获取末端执行器在 Pinocchio 模型中的 frame 索引。
   *
   * @return pinocchio::FrameIndex 末端 frame 的内部索引。
   */
  pinocchio::FrameIndex endEffectorFrameId() const;

  /**
   * @brief 获取只读 Pinocchio 模型对象。
   *
   * @return const pinocchio::Model& 机器人模型引用。
   */
  const pinocchio::Model & model() const;

  /**
   * @brief 获取可写 Pinocchio 模型对象。
   *
   * 仅在确实需要直接操作底层模型时使用。
   *
   * @return pinocchio::Model& 机器人模型引用。
   */
  pinocchio::Model & model();

  /**
   * @brief 获取只读 Pinocchio 数据缓存对象。
   *
   * @return const pinocchio::Data& 数据缓存引用。
   */
  const pinocchio::Data & data() const;

  /**
   * @brief 获取可写 Pinocchio 数据缓存对象。
   *
   * 正逆运动学计算过程中会复用这个缓存，避免重复分配内存。
   *
   * @return pinocchio::Data& 数据缓存引用。
   */
  pinocchio::Data & data();

  /**
   * @brief 获取活动关节名称列表。
   *
   * 返回顺序与关节向量 `q` 中的顺序保持一致。
   *
   * @return const std::vector<std::string>& 关节名称数组。
   */
  const std::vector<std::string> & jointNames() const;

  /**
   * @brief 获取每个关节的位置下限。
   *
   * @return const Eigen::VectorXd& 位置下限向量。
   */
  const Eigen::VectorXd & lowerPositionLimits() const;

  /**
   * @brief 获取每个关节的位置上限。
   *
   * @return const Eigen::VectorXd& 位置上限向量。
   */
  const Eigen::VectorXd & upperPositionLimits() const;

  /**
   * @brief 获取每个关节的速度上限。
   *
   * @return const Eigen::VectorXd& 速度上限向量。
   */
  const Eigen::VectorXd & velocityLimits() const;

  /**
   * @brief 获取模型的中性位姿。
   *
   * 通常可作为逆运动学迭代的初始值。
   * 对不同关节类型，Pinocchio 会按各自定义生成 neutral 状态，
   * 所以它比“简单全零向量”更通用。
   *
   * @return Eigen::VectorXd 中性关节配置。
   */
  Eigen::VectorXd neutralConfiguration() const;

  /**
   * @brief 将输入关节角限制在模型允许范围内。
   *
   * 这个接口不会修改输入本身，而是返回一个新的限幅后向量。
   * 上层常把它用于：
   * 1. IK 每轮迭代后的安全收敛；
   * 2. 手工构造测试姿态时的越界保护。
   *
   * @param configuration 输入关节配置。
   * @return Eigen::VectorXd 限幅后的关节配置。
   */
  Eigen::VectorXd clampToLimits(const Eigen::VectorXd & configuration) const;

  /**
   * @brief 检查关节配置是否合法。
   *
   * 判定条件包括：
   * 1. 模型已加载；
   * 2. 输入维度与模型自由度一致；
   * 3. 每个关节都在允许的上下限范围内。
   *
   * @param configuration 待检查的关节配置。
   * @return true 配置合法。
   * @return false 配置非法。
   */
  bool isConfigurationValid(const Eigen::VectorXd & configuration) const;

  /**
   * @brief 计算末端执行器的正运动学结果，输出为 Pinocchio 的 SE3 位姿。
   *
   * @param joint_positions 输入关节角向量，长度应等于机械臂自由度。
   * @param end_effector_pose 输出末端位姿，包含旋转矩阵和平移向量。
   * @return true 计算成功。
   * @return false 输入维度错误、模型未加载或关节角超限。
   */
  bool forwardKinematics(const Eigen::VectorXd & joint_positions,
                         pinocchio::SE3 & end_effector_pose);

  /**
   * @brief 计算末端执行器的正运动学结果，输出为 ROS Pose 消息。
   *
   * @param joint_positions 输入关节角向量，长度应等于机械臂自由度。
   * @param end_effector_pose 输出末端位姿，位置和四元数都写入该消息。
   * @return true 计算成功。
   * @return false 输入维度错误、模型未加载或关节角超限。
   */
  bool forwardKinematics(const Eigen::VectorXd & joint_positions,
                         geometry_msgs::msg::Pose & end_effector_pose);

  /**
   * @brief 计算任意指定 frame 的正运动学结果，输出为 Pinocchio SE3 位姿。
   *
   * 这个接口相比只支持末端执行器的 `forwardKinematics()` 更通用，
   * 便于做链路中间 link 的避障、可视化和调试。
   *
   * @param joint_positions 输入关节角向量。
   * @param frame_name 目标 frame 名称。
   * @param frame_pose 输出该 frame 相对于世界坐标系的 SE3 位姿。
   * @return true 计算成功。
   * @return false 输入非法或 frame 不存在。
   */
  bool framePose(const Eigen::VectorXd & joint_positions,
                 const std::string & frame_name,
                 pinocchio::SE3 & frame_pose);

  /**
   * @brief 计算任意指定 frame 的正运动学结果，输出为 ROS Pose 消息。
   *
   * @param joint_positions 输入关节角向量。
   * @param frame_name 目标 frame 名称。
   * @param frame_pose 输出 ROS Pose。
   * @return true 计算成功。
   * @return false 输入非法或 frame 不存在。
   */
  bool framePose(const Eigen::VectorXd & joint_positions,
                 const std::string & frame_name,
                 geometry_msgs::msg::Pose & frame_pose);

  /**
   * @brief 检查指定 frame 是否存在于当前 Pinocchio 模型中。
   *
   * @param frame_name 待检查的 frame 名称。
   * @return true frame 存在。
   * @return false frame 不存在。
   */
  bool hasFrame(const std::string & frame_name) const;

  /**
   * @brief 获取指定 frame 在 Pinocchio 模型中的索引。
   *
   * @param frame_name frame 名称。
   * @return pinocchio::FrameIndex 对应索引。
   * @throws std::runtime_error 当 frame 不存在时抛出异常。
   */
  pinocchio::FrameIndex frameId(const std::string & frame_name) const;

  /**
   * @brief 计算指定 frame 的 6 x nv Jacobian。
   *
   * 这里允许调用方选择 Jacobian 的参考坐标系，默认使用
   * `LOCAL_WORLD_ALIGNED`，它很适合做位置避障和笛卡尔控制。
   *
   * @param joint_positions 输入关节角向量。
   * @param frame_name 目标 frame 名称。
   * @param jacobian 输出 Jacobian，维度为 `6 x model.nv`。
   * @param reference_frame Jacobian 参考坐标系。
   * @return true 计算成功。
   * @return false 输入非法或 frame 不存在。
   */
  bool frameJacobian(const Eigen::VectorXd & joint_positions,
                     const std::string & frame_name,
                     pinocchio::Data::Matrix6x & jacobian,
                     pinocchio::ReferenceFrame reference_frame =
                       pinocchio::LOCAL_WORLD_ALIGNED);

private:
  /**
   * @brief 根据当前模型刷新关节元数据缓存。
   *
   * 包括关节名称、位置上下限、速度上下限。
   * 之所以单独缓存一份，是为了让上层节点不必每次都直接访问
   * Pinocchio `Model` 里的底层数组和索引结构。
   */
  void refreshJointMetadata();

  /// Pinocchio 的静态模型：描述关节树、frame、限位、惯量等“结构化信息”。
  pinocchio::Model model_;

  /// 与 `model_` 配套的数据缓存：算法每次运行时的结果会写到这里，避免重复分配。
  pinocchio::Data data_;

  /// URDF 文件路径。`loadURDF()` 会根据它重新构建 `model_`。
  std::string urdf_path_;

  /// 末端执行器 frame 名称，通常对应 URDF 中的 link/frame 名。
  std::string end_effector_frame_;

  /// 末端执行器 frame 在 Pinocchio `model_.frames` 中的索引。
  pinocchio::FrameIndex end_effector_frame_id_;

  /// 原子标记，表示 `model_` / `data_` / 元数据缓存是否已准备就绪。
  std::atomic<bool> is_model_loaded_;

  /// 仅保存“活动关节”的名称，顺序与 `q` 中的关节顺序一致，不含 `universe`。
  std::vector<std::string> joint_names_;

  /// 关节位置下限，直接来自 `model_.lowerPositionLimit`。
  Eigen::VectorXd lower_position_limits_;

  /// 关节位置上限，直接来自 `model_.upperPositionLimit`。
  Eigen::VectorXd upper_position_limits_;

  /// 关节速度上限，直接来自 `model_.velocityLimit`。
  Eigen::VectorXd velocity_limits_;
};
}  // namespace robot_core
