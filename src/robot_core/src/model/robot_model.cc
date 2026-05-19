#include "robot_core/model/robot_model.hpp"

/**
 * @file robot_model.cc
 * @brief 实现机器人模型加载、正运动学、frame 查询和 Jacobian 计算。
 *
 * 这里的实现把 Pinocchio 的底层模型操作做了一层更工程化的包装，
 * 让上层模块可以更直接地使用 URDF、关节限位、frame 位姿和 Jacobian。
 */

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <Eigen/Geometry>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace robot_core
{
  RobotModel::RobotModel(const std::string &urdf_path,
                         const std::string &end_effector_frame)
      // 此处先构造一个空 `model_`，再用它初始化匹配的 `data_`。
      // 后续一旦 `loadURDF()` 重新生成了模型，就必须同步重建 `data_`，
      // 因为 `Data` 的内部尺寸和索引布局完全依赖于 `Model`。
      : model_(),
        data_(model_),
        urdf_path_(urdf_path),
        end_effector_frame_(end_effector_frame),
        end_effector_frame_id_(0),
        is_model_loaded_(false)
  {
  }

  bool RobotModel::loadURDF()
  {
    try
    {
      // 从 URDF 构造 Pinocchio 模型。这里会解析：
      // - joint/link 拓扑关系
      // - 各关节类型与自由度
      // - 几何原点、惯量、限位等信息
      pinocchio::urdf::buildModel(urdf_path_, model_);

      // `model_` 已经变化，因此运行时缓存 `data_` 也必须重新创建。
      data_ = pinocchio::Data(model_);

      // 我们的正运动学接口最终要读取某个 frame 的世界位姿，因此需要先确认该 frame 存在。
      // 这里按 BODY 类型查找，适合以末端 link 作为工具坐标系的常见写法。
      if (!model_.existFrame(end_effector_frame_, pinocchio::BODY))
      {
        throw std::runtime_error("End-effector frame not found: " + end_effector_frame_);
      }

      // frame 名称来自 URDF 中的 link/frame 定义，frame id 是 Pinocchio 内部索引。
      end_effector_frame_id_ = model_.getFrameId(end_effector_frame_, pinocchio::BODY);

      // 额外缓存一份上层常用元数据，避免调用方每次都直接访问 Pinocchio 底层数组。
      refreshJointMetadata();
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error loading URDF model: " << e.what() << std::endl;
      is_model_loaded_ = false;
      return false;
    }

    is_model_loaded_ = true;
    return true;
  }

  bool RobotModel::isModelLoaded() const
  {
    return is_model_loaded_.load();
  }

  std::size_t RobotModel::dof() const
  {
    // 对当前七轴机械臂，URDF 中 7 个 revolute joint 都是 1-DoF，
    // 因此配置空间维度 `nq` 正好等于 7。
    // 若未来接入 free-flyer 或球关节，`nq` 可能不再等于“关节数量”。
    return static_cast<std::size_t>(model_.nq);
  }

  const std::string &RobotModel::endEffectorFrame() const
  {
    return end_effector_frame_;
  }

  pinocchio::FrameIndex RobotModel::endEffectorFrameId() const
  {
    return end_effector_frame_id_;
  }

  const pinocchio::Model &RobotModel::model() const
  {
    return model_;
  }

  pinocchio::Model &RobotModel::model()
  {
    return model_;
  }

  const pinocchio::Data &RobotModel::data() const
  {
    return data_;
  }

  pinocchio::Data &RobotModel::data()
  {
    return data_;
  }

  const std::vector<std::string> &RobotModel::jointNames() const
  {
    return joint_names_;
  }

  const Eigen::VectorXd &RobotModel::lowerPositionLimits() const
  {
    return lower_position_limits_;
  }

  const Eigen::VectorXd &RobotModel::upperPositionLimits() const
  {
    return upper_position_limits_;
  }

  const Eigen::VectorXd &RobotModel::velocityLimits() const
  {
    return velocity_limits_;
  }

  Eigen::VectorXd RobotModel::neutralConfiguration() const
  {
    // Pinocchio 会根据关节类型返回“中性位姿”：
    // - 转动/移动关节通常为 0
    // - 复杂关节类型则按库定义的 neutral 规则生成
    return pinocchio::neutral(model_);
  }

  Eigen::VectorXd RobotModel::clampToLimits(const Eigen::VectorXd &configuration) const
  {
    Eigen::VectorXd clamped = configuration;

    /* 这个判断是属于N轴固定基座机械臂的，当是移动机器人和人形机器人时这条语句就不再适用 */
    if (configuration.size() != model_.nq)
    {
      // 维度不对时不做隐式修正，直接返回原始输入，让调用方自己处理错误来源。
      return clamped;
    }

    for (Eigen::Index idx = 0; idx < configuration.size(); ++idx)
    {
      // 逐维限幅，使每个关节都落在 URDF 给出的合法区间内。
      clamped[idx] = std::min(upper_position_limits_[idx],
                              std::max(lower_position_limits_[idx], configuration[idx]));
    }

    return clamped;
  }

  bool RobotModel::isConfigurationValid(const Eigen::VectorXd &configuration) const
  {
    if (!is_model_loaded_.load() || configuration.size() != model_.nq)
    {
      // 只有在模型已加载且输入维度完全匹配时，后续检查才有意义。
      return false;
    }

    constexpr double kTolerance = 1.0e-9;
    for (Eigen::Index idx = 0; idx < configuration.size(); ++idx)
    {
      // 加一个很小的容差，避免浮点误差导致“理论上等于上限”的值被误判越界。
      if (configuration[idx] < lower_position_limits_[idx] - kTolerance ||
          configuration[idx] > upper_position_limits_[idx] + kTolerance)
      {
        return false;
      }
    }

    return true;
  }

  bool RobotModel::forwardKinematics(const Eigen::VectorXd &joint_positions,
                                     pinocchio::SE3 &end_effector_pose)
  {
    if (!isConfigurationValid(joint_positions))
    {
      return false;
    }

    // 第一步：根据输入关节角更新所有 joint 的空间位姿。
    pinocchio::forwardKinematics(model_, data_, joint_positions);

    // 第二步：把 joint 位姿传播到各个 frame。
    // Pinocchio 将 joint 和 frame 分开管理，因此若要读取某个 link/frame 的位姿，
    // 需要在 forwardKinematics 之后显式调用 updateFramePlacements。
    pinocchio::updateFramePlacements(model_, data_);

    // `oMf` 表示“world(origin) 到 frame(frame) 的变换”。
    // 这里取到的就是末端执行器相对于世界坐标系的 SE3 位姿。
    end_effector_pose = data_.oMf[end_effector_frame_id_];
    return true;
  }

  bool RobotModel::forwardKinematics(const Eigen::VectorXd &joint_positions,
                                     geometry_msgs::msg::Pose &end_effector_pose)
  {
    pinocchio::SE3 pose;
    if (!forwardKinematics(joint_positions, pose))
    {
      return false;
    }

    // Pinocchio 内部使用旋转矩阵 + 平移向量表示 SE3，
    // ROS Pose 则要求四元数 + 平移，因此这里做一次格式转换。
    const Eigen::Quaterniond quaternion(pose.rotation());
    end_effector_pose.position.x = pose.translation().x();
    end_effector_pose.position.y = pose.translation().y();
    end_effector_pose.position.z = pose.translation().z();
    end_effector_pose.orientation.x = quaternion.x();
    end_effector_pose.orientation.y = quaternion.y();
    end_effector_pose.orientation.z = quaternion.z();
    end_effector_pose.orientation.w = quaternion.w();
    return true;
  }

  bool RobotModel::framePose(const Eigen::VectorXd &joint_positions,
                             const std::string &frame_name,
                             pinocchio::SE3 &frame_pose)
  {
    if (!isConfigurationValid(joint_positions) || !hasFrame(frame_name))
    {
      return false;
    }

    pinocchio::forwardKinematics(model_, data_, joint_positions);
    pinocchio::updateFramePlacements(model_, data_);
    frame_pose = data_.oMf[frameId(frame_name)];
    return true;
  }

  bool RobotModel::framePose(const Eigen::VectorXd &joint_positions,
                             const std::string &frame_name,
                             geometry_msgs::msg::Pose &frame_pose)
  {
    pinocchio::SE3 pose;
    if (!framePose(joint_positions, frame_name, pose))
    {
      return false;
    }

    const Eigen::Quaterniond quaternion(pose.rotation());
    frame_pose.position.x = pose.translation().x();
    frame_pose.position.y = pose.translation().y();
    frame_pose.position.z = pose.translation().z();
    frame_pose.orientation.x = quaternion.x();
    frame_pose.orientation.y = quaternion.y();
    frame_pose.orientation.z = quaternion.z();
    frame_pose.orientation.w = quaternion.w();
    return true;
  }

  bool RobotModel::hasFrame(const std::string &frame_name) const
  {
    // 这里只做存在性判断，不区分调用方后续想拿的是 pose 还是 Jacobian。
    return model_.existFrame(frame_name);
  }

  pinocchio::FrameIndex RobotModel::frameId(const std::string &frame_name) const
  {
    if (!hasFrame(frame_name))
    {
      throw std::runtime_error("Frame not found: " + frame_name);
    }
    return model_.getFrameId(frame_name);
  }

  bool RobotModel::frameJacobian(const Eigen::VectorXd &joint_positions,
                                 const std::string &frame_name,
                                 pinocchio::Data::Matrix6x &jacobian,
                                 pinocchio::ReferenceFrame reference_frame)
  {
    if (!isConfigurationValid(joint_positions) || !hasFrame(frame_name))
    {
      return false;
    }

    // `forwardKinematics()` 刷新 joint 位姿，
    // `computeJointJacobians()` 刷新各 joint 的雅可比缓存，
    // `updateFramePlacements()` 再把 joint 结果传播到 frame。
    pinocchio::forwardKinematics(model_, data_, joint_positions);
    pinocchio::computeJointJacobians(model_, data_, joint_positions);
    pinocchio::updateFramePlacements(model_, data_);

    // 输出矩阵固定按 6 x nv 分配：
    // 前 3 行是线速度部分，后 3 行是角速度部分。
    jacobian.resize(6, model_.nv);
    jacobian.setZero();
    // `reference_frame` 由调用方决定 Jacobian 在 local/world 哪个坐标系里表达。
    pinocchio::getFrameJacobian(model_, data_, frameId(frame_name), reference_frame, jacobian);
    return true;
  }

  void RobotModel::refreshJointMetadata()
  {
    // `model_.names` 存的是所有 joint 的名字，包含：
    // - id = 0 的 `universe`
    // - 各活动关节
    // - 可能存在的 fixed joint
    //
    // 对上层控制来说，我们通常只关心会在配置向量 `q` 中占维度的活动关节，
    // 因此这里只收集 `nq() > 0` 的 joint 名称。
    joint_names_.clear();
    const auto joint_count = static_cast<pinocchio::JointIndex>(model_.njoints);
    for (pinocchio::JointIndex joint_id = 1; joint_id < joint_count; ++joint_id)
    {
      if (model_.joints[joint_id].nq() > 0)
      {
        joint_names_.push_back(model_.names[joint_id]);
      }
    }

    // 这三组向量的顺序与配置/速度向量一致。
    // 对当前 URDF 中的 7 个转动关节，它们分别对应 iiwa_joint_1 ... iiwa_joint_7。
    lower_position_limits_ = model_.lowerPositionLimit;
    upper_position_limits_ = model_.upperPositionLimit;
    velocity_limits_ = model_.velocityLimit;
  }
} // namespace robot_core
