#include "robot_core/ik_solver.hpp"

/**
 * @file ik_solver.cc
 * @brief 实现基于 Pinocchio 的数值逆运动学求解算法。
 *
 * 当前实现支持两种常见策略：
 * 1. Damped Least Squares，结构简单、稳定性好；
 * 2. Levenberg-Marquardt，可在收敛速度与稳定性之间自适应折中。
 */

#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace robot_core
{
  namespace
  {
    // SE(3) 位姿误差在李代数中是 6 维向量：
    // [dx, dy, dz, dwx, dwy, dwz]^T
    constexpr int kPoseDim = 6;
    constexpr int kPositionDim = 3;

    Eigen::VectorXd makeInitialConfiguration(const robot_core::RobotModel &robot_model,
                                             const Eigen::VectorXd &initial_guess)
    {
      // 先拿到模型配置空间维度，用它判断调用方给的初值是否可用。
      const pinocchio::Model &model = robot_model.model();
      if (initial_guess.size() == model.nq)
      {
        // 维度匹配时，优先使用调用方提供的初值，
        // 但仍要先做一次关节限幅，避免把非法初值直接送进迭代器。
        return robot_model.clampToLimits(initial_guess);
      }

      // 若调用方没有提供合法初值，则退回到模型的中性位姿。
      return robot_model.neutralConfiguration();
    }

    Eigen::Matrix<double, kPoseDim, 1> computePoseError(const pinocchio::SE3 &current_pose,
                                                        const pinocchio::SE3 &target_pose)
    {
      // `delta_pose` 描述的是“从当前末端位姿走到目标位姿”所需的相对变换。
      const pinocchio::SE3 delta_pose = current_pose.actInv(target_pose);
      // `log6()` 把 SE(3) 误差投影到 6 维李代数向量空间，
      // 便于后续与雅可比一起做线性求解。
      return pinocchio::log6(delta_pose).toVector();
    }

    bool isConverged(const Eigen::Matrix<double, kPoseDim, 1> &error,
                     const robot_core::IKOptions &options)
    {
      // 这里把平移和旋转误差拆开判断，
      // 是因为二者单位不同，通常也会配置不同的容差。
      return error.head<3>().norm() <= options.position_tolerance &&
             error.tail<3>().norm() <= options.orientation_tolerance;
    }

    pinocchio::Data::Matrix6x computeAlignedFrameJacobian(const pinocchio::Model &model,
                                                          pinocchio::Data &data,
                                                          const Eigen::VectorXd &q,
                                                          pinocchio::FrameIndex frame_id,
                                                          const pinocchio::SE3 &delta_pose)
    {
      // 先分配一个 6 x nv 的雅可比矩阵，描述末端速度对关节速度的线性映射。
      pinocchio::Data::Matrix6x jacobian(kPoseDim, model.nv);
      jacobian.setZero();
      // 在当前构型 `q` 下计算目标 frame 的局部坐标系雅可比。
      pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, jacobian);

      // `Jlog6` 把雅可比从原始局部表达映射到与误差向量一致的李代数表达。
      Eigen::Matrix<double, kPoseDim, kPoseDim> jlog;
      pinocchio::Jlog6(delta_pose.inverse(), jlog);
      // 这样处理后，`jacobian` 和 `error` 就在同一个线性化空间里了。
      return -jlog * jacobian;
    }

    double squaredErrorNorm(const Eigen::Matrix<double, kPoseDim, 1> &error)
    {
      // LM 中只需要比较“新误差是否比旧误差更小”，
      // 用平方范数可以省掉一次开方，而且比较结果等价。
      return error.squaredNorm();
    }

    Eigen::VectorXd computeJointLimitGradient(const robot_core::RobotModel &robot_model,
                                              const Eigen::VectorXd &q)
    {
      // 用上下限中点作为“自然姿态”的参考中心。
      const Eigen::VectorXd lower = robot_model.lowerPositionLimits();
      const Eigen::VectorXd upper = robot_model.upperPositionLimits();
      const Eigen::VectorXd center = 0.5 * (lower + upper);
      // 半区间长度用于把不同量纲范围的关节归一化到相近尺度。
      const Eigen::VectorXd half_range = 0.5 * (upper - lower);

      // 这里的代价函数是f(q)=0.5*((q[i]-center[i])^2 / half_range[i]^2)，是归一化函数
      // 因此求导后得到的梯度就是gradient[i]=(q[i]-center[i]) / half_range[i]^2。
      Eigen::VectorXd gradient = Eigen::VectorXd::Zero(q.size());
      for (Eigen::Index i = 0; i < q.size(); ++i)
      {
        // 某些模型若存在极小限位范围，这里加一个下界避免除 0 或梯度爆炸。
        const double safe_half_range = std::max(half_range[i], 1.0e-6);
        // 这一项等价于“归一化二次型代价”的梯度，
        // 关节越靠近上下限，梯度就越倾向把它拉回中心。
        gradient[i] = (q[i] - center[i]) / (safe_half_range * safe_half_range);
      }

      return gradient;
    }

    Eigen::VectorXd computeJointLimitNullspaceStep(const robot_core::RobotModel &robot_model,
                                                   const robot_core::IKOptions &options,
                                                   const Eigen::VectorXd &q,
                                                   const Eigen::MatrixXd &jacobian_pseudoinverse,
                                                   const pinocchio::Data::Matrix6x &jacobian)
    {
      if (!options.enable_joint_limit_nullspace ||
          options.joint_limit_nullspace_gain <= 0.0)
      {
        // 未开启该功能时返回零向量，这样主任务更新公式无需再写分支。
        return Eigen::VectorXd::Zero(jacobian.cols());
      }

      // 先在关节空间求出“往中间拉”的原始偏好方向。
      const Eigen::VectorXd gradient = computeJointLimitGradient(robot_model, q);
      // 用雅可比的伪逆把这个偏好方向投影到主任务的零空间，
      // 这样它就不会干扰末端位姿的收敛了。
      const Eigen::MatrixXd nullspace_projector =
          Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) -
          jacobian_pseudoinverse * jacobian;

      // 负梯度方向对应“往关节中间区域回拉”的次任务速度。
      // 学习率*增益*（I - J# * J）*gradient 形成了一个简单的比例控制律
      return -options.step_size * options.joint_limit_nullspace_gain *
             nullspace_projector * gradient;
    }
  }

  IKSolver::IKSolver(std::shared_ptr<RobotModel> robot_model,
                     const IKOptions &options)
      : robot_model_(std::move(robot_model)),
        options_(options)
  {
    // 构造阶段只保存依赖对象和参数配置；
    // 真正的逆运动学求解发生在后续 `solve()` 调用时。
  }

  bool IKSolver::solve(const geometry_msgs::msg::Pose &target_pose,
                       Eigen::VectorXd &solution,
                       const Eigen::VectorXd &initial_guess)
  {
    // ROS 消息版本的接口只是一个便捷包装；
    // 它先把 Pose 转成 Pinocchio 的 SE3，再复用核心求解逻辑。
    return solve(poseToSE3(target_pose), solution, initial_guess);
  }

  bool IKSolver::solve(const pinocchio::SE3 &target_pose,
                       Eigen::VectorXd &solution,
                       const Eigen::VectorXd &initial_guess)
  {
    // 数值 IK 强依赖模型、frame 和限位数据，
    // 因此入口处先做一次对象合法性保护。
    if (!robot_model_ || !robot_model_->isModelLoaded())
    {
      throw std::runtime_error("Robot model must be loaded before solving IK.");
    }

    // 这里相当于一个运行时策略分发器：
    // 同一套外部接口，根据当前配置选择不同的数值求解内核。
    switch (options_.solver_method)
    {
    case IKSolverMethod::DampedLeastSquares:
      return solveDampedLeastSquares(target_pose, solution, initial_guess);
    case IKSolverMethod::LevenbergMarquardt:
      return solveLevenbergMarquardt(target_pose, solution, initial_guess);
    default:
      throw std::runtime_error("Unsupported IK solver method.");
    }
  }

  bool IKSolver::solvePositionOnly(const geometry_msgs::msg::Pose &target_pose,
                                   Eigen::VectorXd &solution,
                                   const Eigen::VectorXd &initial_guess)
  {
    if (!robot_model_ || !robot_model_->isModelLoaded())
    {
      throw std::runtime_error("Robot model must be loaded before solving IK.");
    }

    const Eigen::Vector3d target_position(
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);
    return solvePositionOnlyDampedLeastSquares(target_position, solution, initial_guess);
  }

  bool IKSolver::solveDampedLeastSquares(const pinocchio::SE3 &target_pose,
                                         Eigen::VectorXd &solution,
                                         const Eigen::VectorXd &initial_guess)
  {
    // 从底层模型里拿到静态结构信息，例如自由度数量和 frame 索引体系。
    const pinocchio::Model &model = robot_model_->model();
    // `data` 用来复用正运动学、frame placement 和雅可比的中间缓存。
    pinocchio::Data &data = robot_model_->data();

    // 默认从中性位姿起步；若调用方给了合法长度的初值，就优先使用初值。
    // 对冗余机械臂来说，初值往往会显著影响最终收敛到哪一支解。
    Eigen::VectorXd q = makeInitialConfiguration(*robot_model_, initial_guess);

    for (int iteration = 0; iteration < options_.max_iterations; ++iteration)
    {
      // 每一轮都基于“当前 q”重新线性化，
      // 这也是数值 IK 区别于一次性解析解的核心特点。
      // 每轮迭代都先根据当前关节角刷新末端位姿。
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      // 使用当前末端位姿到目标位姿的 SE(3) 对数映射作为 6 维误差：
      // 前 3 维是平移误差，后 3 维是旋转误差。
      const pinocchio::SE3 &current_pose = data.oMf[robot_model_->endEffectorFrameId()];
      // `delta_pose` 只作为雅可比表达对齐时的中间量使用。
      const pinocchio::SE3 delta_pose = current_pose.actInv(target_pose);
      const Eigen::Matrix<double, kPoseDim, 1> error = computePoseError(current_pose, target_pose);

      if (isConverged(error, options_))
      {
        // 误差同时满足位置和姿态阈值时，认为求解成功。
        solution = q;
        return true;
      }

      // 计算末端 frame 关于当前关节变量的 6xnv 雅可比矩阵。
      // 这里使用 LOCAL 表达，即雅可比在末端局部坐标系下表示。
      const pinocchio::Data::Matrix6x jacobian =
          computeAlignedFrameJacobian(model, data, q, robot_model_->endEffectorFrameId(), delta_pose);

      // 阻尼最小二乘：
      // delta_q = -alpha * J^T * (J J^T + lambda I)^-1 * error
      //
      // 相比直接求伪逆，这种形式在奇异点附近更稳定。
      const Eigen::Matrix<double, kPoseDim, kPoseDim> system =
          jacobian * jacobian.transpose() +
          options_.damping * Eigen::Matrix<double, kPoseDim, kPoseDim>::Identity();
      // `LDLT` 适合这种对称正定/半正定结构，数值上比直接求逆更稳定。
      const Eigen::LDLT<Eigen::Matrix<double, kPoseDim, kPoseDim>> system_ldlt(system);
      // 这里显式构造阻尼伪逆 `J# = J^T (J J^T + lambda I)^-1`，
      // 既方便算主任务步长，也方便后面构造零空间投影矩阵。
      const Eigen::MatrixXd jacobian_pseudoinverse =
          jacobian.transpose() *
          system_ldlt.solve(Eigen::Matrix<double, kPoseDim, kPoseDim>::Identity());
      // 主任务项只负责减少末端位姿误差，不关心关节姿态是否“好看”。
      const Eigen::VectorXd primary_step =
          -options_.step_size * jacobian_pseudoinverse * error;
      // 次任务项经过零空间投影后，理论上不会破坏主任务的一阶收敛方向。
      const Eigen::VectorXd delta_q =
          primary_step +
          computeJointLimitNullspaceStep(*robot_model_, options_, q, jacobian_pseudoinverse, jacobian);

      // `integrate()` 而不是简单 `q + delta_q`，这样可兼容更一般的关节类型。
      q = pinocchio::integrate(model, q, delta_q);

      // 每次迭代后再做一次限幅，避免数值更新把关节推到模型允许范围外。
      q = robot_model_->clampToLimits(q);
    }

    // 迭代上限耗尽时，返回最后一次迭代结果，便于调试收敛趋势。
    solution = q;
    return false;
  }

  bool IKSolver::solveLevenbergMarquardt(const pinocchio::SE3 &target_pose,
                                         Eigen::VectorXd &solution,
                                         const Eigen::VectorXd &initial_guess)
  {
    // 取出机器人模型本体，后续需要它的自由度、关节拓扑和 frame 索引信息。
    const pinocchio::Model &model = robot_model_->model();
    // 取出与该模型配套的运行时缓存，用来存放正运动学和雅可比的中间结果。
    pinocchio::Data &data = robot_model_->data();

    // 先确定 LM 迭代的起点：
    // - 如果调用方给了合法长度的初值，就从该初值开始；
    // - 否则从机械臂的中性位姿开始。
    Eigen::VectorXd q = makeInitialConfiguration(*robot_model_, initial_guess);
    // `damping` 就是 LM 里的阻尼系数 lambda。
    // 它会在迭代过程中根据“这一步是否有效”自动调整。
    double damping = options_.lm_initial_damping;

    // LM 本质上仍是迭代法，因此我们按最大迭代次数循环逼近目标位姿。
    for (int iteration = 0; iteration < options_.max_iterations; ++iteration)
    {
      // LM 同样是局部迭代法，所以每一步都要先更新当前线性化点。
      // 先根据当前关节角 `q` 计算所有关节和 frame 的空间位姿。
      pinocchio::forwardKinematics(model, data, q);
      // `forwardKinematics()` 只更新 joint 相关结果；
      // 若要读取末端 frame 位姿，还需要显式刷新 frame placement。
      pinocchio::updateFramePlacements(model, data);

      // 取出当前末端执行器在世界坐标系下的位姿。
      const pinocchio::SE3 &current_pose = data.oMf[robot_model_->endEffectorFrameId()];
      // 计算“当前位姿到目标位姿”的相对变换，用于后续误差线性化。
      const pinocchio::SE3 delta_pose = current_pose.actInv(target_pose);
      // 把 SE(3) 位姿误差映射成 6 维李代数向量：
      // 前 3 维对应位置误差，后 3 维对应姿态误差。
      const Eigen::Matrix<double, kPoseDim, 1> error = computePoseError(current_pose, target_pose);

      // 如果当前位置已经满足误差阈值，就认为 LM 收敛成功。
      if (isConverged(error, options_))
      {
        // 输出当前关节解并返回成功。
        solution = q;
        return true;
      }

      // 在当前关节角附近计算末端位姿对关节变量的局部线性映射，也就是雅可比矩阵。
      const pinocchio::Data::Matrix6x jacobian =
          computeAlignedFrameJacobian(model, data, q, robot_model_->endEffectorFrameId(), delta_pose);

      // LM 的核心思想可以理解为：
      // - 当 lambda 很小时，更接近 Gauss-Newton，步子更激进；
      // - 当 lambda 很大时，更接近梯度下降，步子更保守。
      //
      // 这里构造的是法方程矩阵 `J^T J + lambda I`，
      // 其中 `lambda I` 负责在病态或奇异附近提供额外稳定性。
      const Eigen::MatrixXd normal_matrix =
          jacobian.transpose() * jacobian +
          damping * Eigen::MatrixXd::Identity(model.nv, model.nv);
      // 解这个线性系统，得到本轮建议的关节增量 `delta_q`。
      // 外面的 `step_size` 额外控制整体步长，避免一次走得过猛。
      // 这里的法方程维度是 `nv x nv`，更贴近“在关节空间里求增量”的视角。
      const Eigen::LDLT<Eigen::MatrixXd> normal_ldlt(normal_matrix);
      const Eigen::VectorXd primary_step =
          -options_.step_size * normal_ldlt.solve(jacobian.transpose() * error);
      // `solve(J^T)` 得到的是阻尼形式下的广义逆映射，
      // 后面用它来生成 `N = I - J#J`。
      const Eigen::MatrixXd jacobian_pseudoinverse =
          normal_ldlt.solve(jacobian.transpose());
      const Eigen::VectorXd delta_q =
          primary_step +
          computeJointLimitNullspaceStep(*robot_model_, options_, q, jacobian_pseudoinverse, jacobian);

      // 用 Pinocchio 的 `integrate()` 把增量应用到当前配置上，
      // 这样比直接做 `q + delta_q` 更适合一般流形关节。
      Eigen::VectorXd candidate_q = pinocchio::integrate(model, q, delta_q);
      // 再把试探解限制回关节上下界内，避免越界。
      candidate_q = robot_model_->clampToLimits(candidate_q);

      // 对这个“试探解”重新做一次正运动学，看看它带来的末端位姿是否更接近目标。
      pinocchio::forwardKinematics(model, data, candidate_q);
      // 同样需要刷新 frame 位姿，才能读取新的末端位姿。
      pinocchio::updateFramePlacements(model, data);

      // 取出试探解对应的末端位姿。
      const pinocchio::SE3 &candidate_pose = data.oMf[robot_model_->endEffectorFrameId()];
      // 再把试探解的位姿偏差转成 6 维误差向量。
      const Eigen::Matrix<double, kPoseDim, 1> candidate_error =
          computePoseError(candidate_pose, target_pose);

      // LM 的关键就在这里：
      // 如果新解确实让误差变小，就接受这一步，并把阻尼调小一点，
      // 让后续迭代更接近快速的 Gauss-Newton。
      if (squaredErrorNorm(candidate_error) < squaredErrorNorm(error))
      {
        // 接受这次试探步，把当前解更新为 candidate_q。
        q = candidate_q;
        // 误差下降说明当前局部二次近似比较可信，可以适当减小 lambda。
        // 这里还用 `options_.damping` 作为下界，避免阻尼无限趋近于 0。
        damping = std::max(options_.damping, damping * options_.lm_decrease_factor);
      }
      else
      {
        // 如果误差没有下降，就拒绝这一步，不更新 q。
        // 同时把阻尼放大，让下一轮更保守、更像梯度下降。
        damping *= options_.lm_increase_factor;
      }
    }

    // 若达到最大迭代次数仍未满足阈值，则返回最后一次保留下来的关节解。
    solution = q;
    return false;
  }

  bool IKSolver::solvePositionOnlyDampedLeastSquares(const Eigen::Vector3d &target_position,
                                                     Eigen::VectorXd &solution,
                                                     const Eigen::VectorXd &initial_guess)
  {
    const pinocchio::Model &model = robot_model_->model();
    pinocchio::Data &data = robot_model_->data();
    Eigen::VectorXd q = makeInitialConfiguration(*robot_model_, initial_guess);

    for (int iteration = 0; iteration < options_.max_iterations; ++iteration)
    {
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      const pinocchio::SE3 &current_pose = data.oMf[robot_model_->endEffectorFrameId()];
      const Eigen::Vector3d position_error = target_position - current_pose.translation();
      if (position_error.norm() <= options_.position_tolerance)
      {
        solution = q;
        return true;
      }

      pinocchio::Data::Matrix6x full_jacobian(kPoseDim, model.nv);
      full_jacobian.setZero();
      pinocchio::computeFrameJacobian(
          model,
          data,
          q,
          robot_model_->endEffectorFrameId(),
          pinocchio::LOCAL_WORLD_ALIGNED,
          full_jacobian);

      const Eigen::MatrixXd position_jacobian = full_jacobian.topRows(kPositionDim);
      const Eigen::Matrix3d system =
          position_jacobian * position_jacobian.transpose() +
          options_.damping * Eigen::Matrix3d::Identity();
      const Eigen::LDLT<Eigen::Matrix3d> system_ldlt(system);
      const Eigen::MatrixXd jacobian_pseudoinverse =
          position_jacobian.transpose() *
          system_ldlt.solve(Eigen::Matrix3d::Identity());
      const Eigen::VectorXd primary_step =
          options_.step_size * jacobian_pseudoinverse * position_error;
      const Eigen::MatrixXd nullspace_projector =
          Eigen::MatrixXd::Identity(model.nv, model.nv) -
          jacobian_pseudoinverse * position_jacobian;
      Eigen::VectorXd joint_limit_step = Eigen::VectorXd::Zero(model.nv);
      if (options_.enable_joint_limit_nullspace &&
          options_.joint_limit_nullspace_gain > 0.0)
      {
        joint_limit_step =
            -options_.step_size * options_.joint_limit_nullspace_gain *
            nullspace_projector *
            computeJointLimitGradient(*robot_model_, q);
      }

      q = pinocchio::integrate(model, q, primary_step + joint_limit_step);
      q = robot_model_->clampToLimits(q);
    }

    solution = q;
    return false;
  }

  bool IKSolver::solveCartesianPath(const std::vector<geometry_msgs::msg::Pose> &cartesian_waypoints,
                                    std::vector<Eigen::VectorXd> &joint_waypoints,
                                    const Eigen::VectorXd &initial_guess)
  {
    joint_waypoints.clear();

    // 空路径没有“逐点求解”的意义，因此直接返回失败。
    if (cartesian_waypoints.empty())
    {
      return false;
    }

    // 第一帧的 IK 初值由调用方决定；
    // 若没给合法初值，就从中性位姿开始。
    Eigen::VectorXd seed =
        initial_guess.size() == static_cast<Eigen::Index>(robot_model_->dof()) ? initial_guess : robot_model_->neutralConfiguration();

    // 顺序求解整条笛卡尔路径，并把“上一个点的逆解”作为“下一个点的初值”。
    // 这样通常能让相邻路径点的关节解更连续，也更容易收敛。
    for (const auto &waypoint : cartesian_waypoints)
    {
      // 每个路径点单独做一次单点 IK，
      // 但 seed 会沿路径逐段传递，从而提高连续性。
      Eigen::VectorXd solution;
      if (!solve(waypoint, solution, seed))
      {
        return false;
      }

      joint_waypoints.push_back(solution);
      seed = solution;
    }

    return true;
  }

  bool IKSolver::isPoseReachable(const geometry_msgs::msg::Pose &target_pose,
                                 Eigen::VectorXd *solution,
                                 const Eigen::VectorXd &initial_guess)
  {
    // 这里直接复用正式的 IK 求解器做“可达性判定”，
    // 这样判定标准和真实求解行为完全一致。
    Eigen::VectorXd candidate;
    const bool reachable = solve(target_pose, candidate, initial_guess);
    if (reachable && solution != nullptr)
    {
      // 调用方若需要，就顺手把找到的关节解返回给它复用。
      *solution = candidate;
    }

    return reachable;
  }

  const IKOptions &IKSolver::options() const
  {
    // 返回只读引用，避免不必要的参数拷贝。
    return options_;
  }

  void IKSolver::setOptions(const IKOptions &options)
  {
    // 直接整体替换当前参数集合，后续求解会立刻使用新配置。
    options_ = options;
  }

  IKSolverMethod IKSolver::solverMethod() const
  {
    // 便于上层在运行时查询当前究竟启用了哪种 IK 算法。
    return options_.solver_method;
  }

  void IKSolver::setSolverMethod(IKSolverMethod method)
  {
    // 只切换算法类型，其余容差、步长和阻尼参数保持不变。
    options_.solver_method = method;
  }

  pinocchio::SE3 IKSolver::poseToSE3(const geometry_msgs::msg::Pose &pose)
  {
    // ROS Pose 用四元数表示姿态；Pinocchio SE3 内部则使用旋转矩阵 + 平移。
    Eigen::Quaterniond quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    quaternion.normalize();

    // 先把四元数归一化，再转旋转矩阵，
    // 可以避免外部传入的非单位四元数把姿态误差放大。
    return pinocchio::SE3(
        quaternion.toRotationMatrix(),
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
  }

  geometry_msgs::msg::Pose IKSolver::se3ToPose(const pinocchio::SE3 &pose)
  {
    // 这里做的是 Pinocchio 内部表示到 ROS 消息表示的纯格式变换。
    geometry_msgs::msg::Pose ros_pose;

    // 旋转矩阵转四元数时，Eigen 会自动生成单位四元数。
    const Eigen::Quaterniond quaternion(pose.rotation());

    ros_pose.position.x = pose.translation().x();
    ros_pose.position.y = pose.translation().y();
    ros_pose.position.z = pose.translation().z();
    ros_pose.orientation.x = quaternion.x();
    ros_pose.orientation.y = quaternion.y();
    ros_pose.orientation.z = quaternion.z();
    ros_pose.orientation.w = quaternion.w();
    // 返回的 Pose 可直接给上层节点发布、记录或再次作为 IK 目标输入。
    return ros_pose;
  }
} // namespace robot_core
