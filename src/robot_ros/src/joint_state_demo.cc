#include "robot_ros/joint_state_demo.hpp"

/**
 * @file joint_state_demo.cc
 * @brief 实现用于 RViz 验证的关节状态演示节点。
 */

#include <chrono>
#include <functional>
#include <stdexcept>

#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robot_ros
{
  JointStateDemoNode::JointStateDemoNode()
      : rclcpp::Node("joint_state_demo"),
        startup_time_(std::chrono::steady_clock::now())
  {
    // 第一步先把所有运行期参数读进来，
    // 后面建模、IK、轨迹规划都会依赖这些参数。
    declareParameters();
    // 参数准备好之后，再根据 URDF 路径和末端 frame 构建机器人模型。
    initializeRobotModel();

    // 轨迹规划器负责把关节路径点离散成可发布的轨迹采样点。
    trajectory_planner_ = std::make_unique<robot_core::JointTrajectoryPlanner>(
        robot_model_, sample_period_);
    // IK 求解器负责把笛卡尔路径点转成关节路径点。
    ik_solver_ = std::make_unique<robot_core::IKSolver>(robot_model_);

    // 这里一次性构造完整演示轨迹，后续定时器只负责顺序播放。
    buildTrajectory();
    // 发布器把内部轨迹点转换成 ROS `joint_states` 话题给 RViz 消费。
    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    // 发布周期直接跟轨迹采样周期保持一致，这样 RViz 中的关节运动节奏与规划结果一致。
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(sample_period_)),
        std::bind(&JointStateDemoNode::publishNextState, this));

    RCLCPP_INFO(
        get_logger(),
        "Joint state demo is ready. Publishing %zu samples from URDF: %s",
        trajectory_.size(),
        urdf_path_.c_str());
  }

  void JointStateDemoNode::declareParameters()
  {
    // 这些参数既提供默认值，也允许 launch/YAML 在启动时覆盖。
    urdf_path_ = declare_parameter<std::string>("urdf_path", resolveDefaultUrdfPath());
    end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "iiwa_link_ee");
    loop_trajectory_ = declare_parameter<bool>("loop_trajectory", true);
    segment_duration_ = declare_parameter<double>("segment_duration", 2.0);
    sample_period_ = declare_parameter<double>("sample_period", 0.05);
    demo_start_delay_ = declare_parameter<double>("demo_start_delay", 0.0);

    // 采样周期必须为正，否则发布定时器和轨迹离散都没有明确物理意义。
    if (sample_period_ <= 0.0)
    {
      throw std::runtime_error("Parameter 'sample_period' must be greater than zero.");
    }

    // 每段轨迹都要有正的持续时间，才能做有效的插值。
    if (segment_duration_ <= 0.0)
    {
      throw std::runtime_error("Parameter 'segment_duration' must be greater than zero.");
    }

    // 延时允许为 0，但不允许是负数。
    if (demo_start_delay_ < 0.0)
    {
      throw std::runtime_error("Parameter 'demo_start_delay' must be non-negative.");
    }
  }

  void JointStateDemoNode::initializeRobotModel()
  {
    // 用当前参数指定的 URDF 路径和末端 frame 构造底层模型对象。
    robot_model_ = std::make_shared<robot_core::RobotModel>(urdf_path_, end_effector_frame_);
    // 真正的 URDF 解析发生在 `loadURDF()` 里；
    // 若这里失败，后续 FK/IK/轨迹规划都没有继续下去的意义。
    if (!robot_model_->loadURDF())
    {
      throw std::runtime_error("Failed to load robot model from URDF: " + urdf_path_);
    }
  }

  std::vector<Eigen::VectorXd> JointStateDemoNode::createDemoWaypoints() const
  {
    // 这里刻意选择几组“幅度适中”的关节目标，方便在 RViz 中观察：
    // - 关节名顺序是否正确
    // - 插值轨迹是否平滑
    // - URDF 限位是否按预期生效
    const Eigen::VectorXd neutral = robot_model_->neutralConfiguration();

    Eigen::VectorXd pose_a = neutral;
    pose_a[1] = 0.3;
    pose_a[3] = 0.4;
    pose_a[5] = 0.5;

    Eigen::VectorXd pose_b = neutral;
    pose_b[0] = -0.7;
    pose_b[2] = 0.9;
    pose_b[4] = -0.6;
    pose_b[6] = 1.0;

    Eigen::VectorXd pose_c = neutral;
    pose_c[0] = 0.5;
    pose_c[2] = -0.7;
    pose_c[4] = 0.8;
    pose_c[6] = -0.9;

    return {
        robot_model_->clampToLimits(neutral),
        robot_model_->clampToLimits(pose_a),
        robot_model_->clampToLimits(pose_b),
        robot_model_->clampToLimits(pose_c),
        // robot_model_->clampToLimits(neutral)
    };
  }

  std::vector<geometry_msgs::msg::Pose> JointStateDemoNode::createDemoCartesianWaypoints() const
  {
    // 为了先稳定验证“笛卡尔点 -> IK -> 轨迹规划”整条链路，
    // 这里不再手写可能超工作空间的目标位姿，而是：
    // 1. 先构造一组已知可达的关节配置；
    // 2. 再用正运动学把它们转换成末端笛卡尔位姿。
    //
    // 这样得到的每个 Pose 都来自当前机械臂真实可达的关节姿态，
    // 更适合拿来做 IK 回归测试。
    const Eigen::VectorXd neutral = robot_model_->neutralConfiguration();
    std::vector<Eigen::VectorXd> joint_samples;
    joint_samples.reserve(8);

    joint_samples.push_back(robot_model_->clampToLimits(neutral));

    Eigen::VectorXd sample_a = neutral;
    sample_a[0] = 0.20;
    sample_a[1] = 0.35;
    sample_a[3] = -0.45;
    sample_a[5] = 0.30;
    joint_samples.push_back(robot_model_->clampToLimits(sample_a));

    Eigen::VectorXd sample_b = neutral;
    sample_b[0] = -0.25;
    sample_b[2] = 0.40;
    sample_b[4] = -0.30;
    sample_b[6] = 0.35;
    joint_samples.push_back(robot_model_->clampToLimits(sample_b));

    Eigen::VectorXd sample_c = neutral;
    sample_c[1] = 0.55;
    sample_c[2] = -0.35;
    sample_c[3] = -0.60;
    sample_c[5] = 0.45;
    joint_samples.push_back(robot_model_->clampToLimits(sample_c));

    Eigen::VectorXd sample_d = neutral;
    sample_d[0] = 0.30;
    sample_d[2] = 0.55;
    sample_d[4] = 0.25;
    sample_d[6] = -0.40;
    joint_samples.push_back(robot_model_->clampToLimits(sample_d));

    Eigen::VectorXd sample_e = neutral;
    sample_e[1] = -0.30;
    sample_e[3] = 0.40;
    sample_e[5] = -0.35;
    sample_e[6] = 0.25;
    joint_samples.push_back(robot_model_->clampToLimits(sample_e));

    Eigen::VectorXd sample_f = neutral;
    sample_f[0] = -0.35;
    sample_f[1] = 0.20;
    sample_f[2] = -0.45;
    sample_f[4] = 0.30;
    joint_samples.push_back(robot_model_->clampToLimits(sample_f));

    Eigen::VectorXd sample_g = neutral;
    sample_g[0] = 0.15;
    sample_g[2] = 0.25;
    sample_g[3] = -0.35;
    sample_g[5] = 0.20;
    sample_g[6] = -0.30;
    joint_samples.push_back(robot_model_->clampToLimits(sample_g));

    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    cartesian_waypoints.reserve(joint_samples.size());

    for (std::size_t idx = 0; idx < joint_samples.size(); ++idx)
    {
      // 先为当前关节样本准备一个 ROS Pose 容器。
      geometry_msgs::msg::Pose pose;
      // 用正运动学把“已知可达的关节姿态”转换成“对应的末端笛卡尔位姿”。
      if (!robot_model_->forwardKinematics(joint_samples[idx], pose))
      {
        throw std::runtime_error("Failed to generate demo Cartesian waypoint from forward kinematics.");
      }

      // 这样得到的位姿天然与当前模型一致，适合作为 IK 回归测试样本。
      cartesian_waypoints.push_back(pose);
    }

    return cartesian_waypoints;
  }

  bool JointStateDemoNode::isCartesianWaypointReachable(const geometry_msgs::msg::Pose &pose,
                                                        Eigen::VectorXd *solution,
                                                        const Eigen::VectorXd &initial_guess)
  {
    // 先确保节点初始化阶段已经成功创建 IK 求解器。
    if (!ik_solver_)
    {
      throw std::runtime_error("IK solver must be initialized before workspace checking.");
    }

    // 可达性检查内部本质上就是“尝试做一次 IK，看能否在误差阈值内收敛”。
    return ik_solver_->isPoseReachable(pose, solution, initial_guess);
  }

  void JointStateDemoNode::buildTrajectory()
  {
    // 创建一组笛卡尔空间的路径点，作为演示用的末端目标位姿序列。
    const std::vector<geometry_msgs::msg::Pose> cartesian_waypoints = createDemoCartesianWaypoints();
    // `waypoints` 用来收集每个笛卡尔路径点对应的关节逆解。
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.reserve(cartesian_waypoints.size());

    // 第一帧 IK 从中性位姿起步，后续每一帧都沿用上一帧的解作为初值。
    Eigen::VectorXd seed = robot_model_->neutralConfiguration();
    for (std::size_t idx = 0; idx < cartesian_waypoints.size(); ++idx)
    {
      // `solution` 存放当前这个笛卡尔路径点求出来的关节逆解。
      Eigen::VectorXd solution;
      // 在构建演示轨迹前，先逐点检查每个笛卡尔路径点的可达性，并收集对应的关节解。
      if (!isCartesianWaypointReachable(cartesian_waypoints[idx], &solution, seed))
      {
        const auto &pose = cartesian_waypoints[idx];
        RCLCPP_ERROR(
            get_logger(),
            "Cartesian waypoint %zu is not reachable: position=(%.3f, %.3f, %.3f), "
            "orientation=(%.3f, %.3f, %.3f, %.3f)",
            idx,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        throw std::runtime_error("Failed workspace check for demo Cartesian waypoint.");
      }

      // 当前路径点求解成功后，把它收进关节路径数组。
      waypoints.push_back(solution);
      // 同时把这次逆解作为下一次 IK 的初值，提升连续性和收敛性。
      seed = solution;
    }

    if (waypoints.size() < 2U)
    {
      throw std::runtime_error("Not enough reachable joint waypoints were generated from IK.");
    }

    if (!trajectory_planner_->planWaypoints(
            waypoints, segment_duration_, trajectory_, sample_period_))
    {
      // 若关节路径无法被顺利离散成轨迹，说明 IK 到轨迹规划这条链路没有完全打通。
      throw std::runtime_error("Failed to build demo joint trajectory.");
    }

    if (trajectory_.empty())
    {
      throw std::runtime_error("Generated trajectory is empty.");
    }
  }

  void JointStateDemoNode::publishCurrentState(const std::size_t trajectory_index)
  {
    // 越界保护：没有轨迹或索引非法时直接跳过本次发布。
    if (trajectory_.empty() || trajectory_index >= trajectory_.size())
    {
      return;
    }

    // 内部轨迹点先转换成 ROS `JointState`，再补上当前时间戳。
    sensor_msgs::msg::JointState joint_state =
        trajectory_planner_->toJointState(trajectory_[trajectory_index]);
    joint_state.header.stamp = now();
    // 发布出去后，robot_state_publisher / RViz 就能据此更新机器人姿态。
    joint_state_publisher_->publish(joint_state);
  }

  void JointStateDemoNode::publishNextState()
  {
    // 没有轨迹就不做任何事，避免定时器空转时报错。
    if (trajectory_.empty())
    {
      return;
    }

    // 用单调时钟计算从节点启动到现在过去了多久。
    const std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now() - startup_time_;
    if (elapsed.count() < demo_start_delay_)
    {
      // 在正式播放前持续发布起始姿态，让 robot_state_publisher 尽早建立完整 TF 树。
      publishCurrentState(0U);
      return;
    }

    if (trajectory_index_ >= trajectory_.size())
    {
      if (!loop_trajectory_)
      {
        // 不循环时停在最后一个姿态；robot_state_publisher 会继续保持最后一次 TF。
        publish_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Trajectory playback completed.");
        return;
      }

      // 允许循环时，重新从轨迹开头开始播放。
      trajectory_index_ = 0U;
    }

    // 先发布当前索引对应的轨迹点。
    publishCurrentState(trajectory_index_);
    // 再把索引推进到下一帧，等待下一次定时器触发。
    ++trajectory_index_;
  }

  std::string JointStateDemoNode::resolveDefaultUrdfPath() const
  {
    // 通过 ament index 找到 `robot_description` 包安装后的 share 目录。
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("robot_description");
    // 再拼出默认 URDF 文件路径，供未显式传参时使用。
    return package_share + "/urdf/robot.urdf";
  }
} // namespace robot_ros
