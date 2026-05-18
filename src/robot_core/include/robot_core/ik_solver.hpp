#pragma once

/**
 * @file ik_solver.hpp
 * @brief 定义基于 Pinocchio 的数值逆运动学求解器接口。
 *
 * 该文件提供单点位姿 IK、笛卡尔路径逐点 IK 以及相关配置结构，
 * 供上层节点或规划器把笛卡尔空间目标转换成关节空间解。
 */

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "robot_core/robot_model.hpp"

namespace robot_core
{
  /**
   * @brief 数值逆运动学求解器类型。
   *
   * 这里的“方法选择”只影响每一轮迭代如何根据误差和雅可比计算 `delta_q`，
   * 不影响外部接口的输入输出形式。
   */
  enum class IKSolverMethod
  {
    /// 阻尼最小二乘法，形式简单，行为稳定。
    DampedLeastSquares,

    /// Levenberg-Marquardt，自适应调节阻尼，更适合兼顾收敛速度与稳定性。
    LevenbergMarquardt,
  };

  /**
   * @brief 逆运动学求解参数集合。
   *
   * 这些参数控制 Pinocchio 数值迭代 IK 的收敛速度和稳定性。
   */
  struct IKOptions
  {
    /// 当前使用的逆运动学数值求解方法。
    /// 默认先选阻尼最小二乘，方便得到更直观、稳定的基础行为。
    IKSolverMethod solver_method{IKSolverMethod::DampedLeastSquares};

    /// 最大迭代次数，超过该次数仍未收敛则返回失败。
    /// 这个上限同时适用于 DLS 和 LM 两种求解方法。
    int max_iterations{500};

    /// 末端位置误差容差，单位为米。
    double position_tolerance{1.0e-4};

    /// 末端姿态误差容差，单位为弧度。
    double orientation_tolerance{1.0e-3};

    /// 阻尼最小二乘中的阻尼系数，用于提升奇异位形附近的稳定性。
    /// 在 LM 中它也会作为最小阻尼下界使用，防止 lambda 过小。
    double damping{1.0e-4};

    /// 每次迭代的积分步长，越大收敛越快，但也更容易震荡。
    double step_size{0.10};

    /// Levenberg-Marquardt 的初始阻尼系数。
    /// 可以理解为 LM 刚开始时更偏向“Gauss-Newton”还是“梯度下降”。
    double lm_initial_damping{1.0e-2};

    /// Levenberg-Marquardt 成功下降时的阻尼缩放系数，应小于 1。
    double lm_decrease_factor{0.5};

    /// Levenberg-Marquardt 未下降时的阻尼放大系数，应大于 1。
    double lm_increase_factor{2.0};

    /// 是否在冗余自由度上启用 joint-limit avoidance 的零空间控制。
    bool enable_joint_limit_nullspace{true};

    /// 关节限位回避项的增益，越大越倾向把关节往中间区域拉回。
    double joint_limit_nullspace_gain{0.05};
  };

  /**
   * @brief 基于 Pinocchio 的数值逆运动学求解器。
   *
   * 这个类使用阻尼最小二乘法对七轴机械臂进行迭代求解，
   * 也支持切换到 Levenberg-Marquardt，
   * 可用于单点位姿 IK 和笛卡尔路径逐点求解。
   *
   * 典型使用流程如下：
   * 1. 先创建并加载 `RobotModel`；
   * 2. 再把模型传给 `IKSolver`；
   * 3. 调用 `solve()` 求单个末端位姿的逆解；
   * 4. 或调用 `solveCartesianPath()` 让一串笛卡尔路径点逐点转成关节路径。
   *
   * 注意这里求得的是“数值解”而不是解析解，因此：
   * - 结果依赖初值 `initial_guess`；
   * - 在奇异位形附近可能收敛变慢；
   * - 对冗余机械臂而言，不同初值可能收敛到不同可行解。
   */
  class IKSolver
  {
  public:
    /**
     * @brief 构造逆运动学求解器。
     *
     * 构造时不会立刻执行任何 IK 计算，
     * 它只是把模型引用和参数集合保存起来，供后续 `solve()` 调用复用。
     *
     * @param robot_model 已加载的机器人模型对象。
     * @param options IK 迭代参数，可使用默认值。
     */
    explicit IKSolver(std::shared_ptr<RobotModel> robot_model,
                      const IKOptions &options = IKOptions{});

    /**
     * @brief 对 ROS Pose 目标位姿执行逆运动学求解。
     *
     * @param target_pose 目标末端位姿。
     * @param solution 输出求解得到的关节角向量。
     * @param initial_guess 逆解初值；若为空，则使用模型中性位姿。
     * @return true 成功收敛到目标附近。
     * @return false 迭代结束后仍未满足误差要求。
     */
    bool solve(const geometry_msgs::msg::Pose &target_pose,
               Eigen::VectorXd &solution,
               const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

    /**
     * @brief 对 Pinocchio SE3 目标位姿执行逆运动学求解。
     *
     * @param target_pose 目标末端位姿。
     * @param solution 输出求解得到的关节角向量。
     * @param initial_guess 逆解初值；若为空，则使用模型中性位姿。
     * @return true 成功收敛到目标附近。
     * @return false 迭代结束后仍未满足误差要求。
     */
    bool solve(const pinocchio::SE3 &target_pose,
               Eigen::VectorXd &solution,
               const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

    /**
     * @brief 仅对目标位置执行逆运动学求解，不约束末端姿态。
     *
     * 该接口适合 Gazebo 中的简单位置控制场景：
     * - 只要求末端到达目标点；
     * - 不要求同时满足某个特定姿态。
     *
     * @param target_pose 目标末端位姿消息，其中仅使用 position 字段。
     * @param solution 输出求解得到的关节角向量。
     * @param initial_guess 逆解初值；若为空，则使用模型中性位姿。
     * @return true 成功收敛到目标位置附近。
     * @return false 迭代结束后仍未满足位置误差要求。
     */
    bool solvePositionOnly(const geometry_msgs::msg::Pose &target_pose,
                           Eigen::VectorXd &solution,
                           const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

    /**
     * @brief 对一组笛卡尔路径点依次求解逆运动学。
     *
     * 每个路径点以上一个点的逆解结果作为下一点的初值，
     * 这样通常能获得更平滑、收敛性更好的关节路径。
     *
     * @param cartesian_waypoints 笛卡尔空间路径点序列。
     * @param joint_waypoints 输出关节空间路径点序列。
     * @param initial_guess 第一个路径点的逆解初值。
     * @return true 所有路径点都求解成功。
     * @return false 任意一个路径点求解失败。
     */
    bool solveCartesianPath(const std::vector<geometry_msgs::msg::Pose> &cartesian_waypoints,
                            std::vector<Eigen::VectorXd> &joint_waypoints,
                            const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

    /**
     * @brief 判断目标位姿是否在当前机械臂的可达工作空间内。
     *
     * 这里的“工作空间判定”不是只看位置半径，而是直接用数值 IK 做可达性检查：
     * - 若某个位姿能在给定误差阈值内收敛，则认为该位姿可达；
     * - 若需要，可将求得的逆解一并返回给调用方复用。
     *
     * @param target_pose 待检查的末端目标位姿。
     * @param solution 若非空，则在可达时写入对应关节解。
     * @param initial_guess 逆解初值；若为空，则使用模型中性位姿。
     * @return true 位姿可达。
     * @return false 位姿不可达或未在迭代次数内收敛。
     */
    bool isPoseReachable(const geometry_msgs::msg::Pose &target_pose,
                         Eigen::VectorXd *solution = nullptr,
                         const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

    /**
     * @brief 获取当前 IK 参数。
     *
     * 当上层想查看当前求解器到底在用哪组容差、阻尼和步长时，
     * 可以通过这个接口拿到完整配置快照。
     *
     * @return const IKOptions& 当前参数配置。
     */
    const IKOptions &options() const;

    /**
     * @brief 更新 IK 参数。
     *
     * 这个接口适合一次性替换整组参数；
     * 如果只想切换算法，可以直接使用 `setSolverMethod()`。
     *
     * @param options 新的 IK 参数配置。
     */
    void setOptions(const IKOptions &options);

    /**
     * @brief 获取当前使用的逆运动学求解方法。
     *
     * @return IKSolverMethod 当前求解方法。
     */
    IKSolverMethod solverMethod() const;

    /**
     * @brief 单独切换逆运动学求解方法。
     *
     * @param method 新的求解方法。
     */
    void setSolverMethod(IKSolverMethod method);

    /**
     * @brief 将 ROS Pose 消息转换为 Pinocchio SE3 位姿。
     *
     * @param pose ROS 位姿消息。
     * @return pinocchio::SE3 对应的 Pinocchio 位姿对象。
     */
    static pinocchio::SE3 poseToSE3(const geometry_msgs::msg::Pose &pose);

    /**
     * @brief 将 Pinocchio SE3 位姿转换为 ROS Pose 消息。
     *
     * @param pose Pinocchio 位姿对象。
     * @return geometry_msgs::msg::Pose 对应的 ROS 位姿消息。
     */
    static geometry_msgs::msg::Pose se3ToPose(const pinocchio::SE3 &pose);

  private:
    /// 使用固定阻尼的阻尼最小二乘法求解单个位姿 IK。
    bool solveDampedLeastSquares(const pinocchio::SE3 &target_pose,
                                 Eigen::VectorXd &solution,
                                 const Eigen::VectorXd &initial_guess);

    /// 使用可自适应阻尼的 Levenberg-Marquardt 求解单个位姿 IK。
    bool solveLevenbergMarquardt(const pinocchio::SE3 &target_pose,
                                 Eigen::VectorXd &solution,
                                 const Eigen::VectorXd &initial_guess);

    /// 使用位置误差和位置 Jacobian 的阻尼最小二乘法求解位置 IK。
    bool solvePositionOnlyDampedLeastSquares(const Eigen::Vector3d &target_position,
                                             Eigen::VectorXd &solution,
                                             const Eigen::VectorXd &initial_guess);

    /// 由外部注入的机器人模型。IK 过程中会直接复用其 `model()` 和 `data()`。
    std::shared_ptr<RobotModel> robot_model_;

    /// 当前求解器参数，例如迭代次数、阻尼和误差阈值。
    IKOptions options_;
  };
} // namespace robot_core
