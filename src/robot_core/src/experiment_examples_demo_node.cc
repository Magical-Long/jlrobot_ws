#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_core/experiment_examples.hpp"

/**
 * @file experiment_examples_demo_node.cc
 * @brief 提供统一的示例节点，用于切换 null-space、势场法和 MoveIt 对照实验。
 */

namespace robot_core
{
/**
 * @brief 统一的实验示例节点。
 *
 * 这个节点位于整个 demo 链路的最上层，负责把：
 * 1. launch / YAML 传入的参数；
 * 2. 机器人模型与各类实验对象；
 * 3. 实验输出的关节轨迹与日志摘要；
 * 串成一条完整的“可直接运行”的 ROS2 节点流程。
 */
class ExperimentExamplesDemoNode : public rclcpp::Node
{
public:
  ExperimentExamplesDemoNode()
  : Node("experiment_examples_demo")
  {
    // 这一组参数描述“模型从哪里来、实验跑哪种模式、结果发到哪个话题”。
    declare_parameter<std::string>("urdf_path", defaultUrdfPath());
    declare_parameter<std::string>("end_effector_frame", "iiwa_link_ee");
    declare_parameter<std::string>("experiment_mode", "nullspace_ik");
    declare_parameter<bool>("use_safe_demo_target", true);
    declare_parameter<std::string>("joint_state_topic", "joint_states");
    declare_parameter<bool>("continuous_random_goal_loop", false);
    declare_parameter<double>("random_goal_period_sec", 5.0);
    declare_parameter<int>("random_goal_attempts_per_cycle", 1);
    declare_parameter<double>("random_goal_joint_limit_margin_ratio", 0.15);
    declare_parameter<double>("random_goal_local_joint_delta_ratio", 0.20);
    declare_parameter<bool>("random_goal_precheck_with_ik", true);
    declare_parameter<bool>("reuse_last_success_as_start", true);

    // 这一组参数描述实验公共输入：
    // - 目标末端位姿；
    // - 起始关节姿态；
    // - 工作空间障碍物。
    declare_parameter<std::vector<double>>("target_position", {0.35, 0.15, 0.65});
    declare_parameter<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "start_configuration", std::vector<double>(7, 0.0));
    declare_parameter<std::vector<std::string>>(
      "collision_frames",
      {"iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4",
       "iiwa_link_5", "iiwa_link_6", "iiwa_link_7", "iiwa_link_ee"});
    declare_parameter<std::vector<double>>(
      "obstacle_centers",
      {0.25, 0.10, 0.45,
       0.32, -0.12, 0.60});
    declare_parameter<std::vector<double>>("obstacle_radii", {0.08, 0.07});
    declare_parameter<std::vector<double>>("obstacle_safety_margins", {0.04, 0.04});
    declare_parameter<double>("segment_duration", 0.15);
    declare_parameter<double>("sample_period", 0.05);
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<int>("cartesian_waypoint_count", 8);

    // 这组参数专门控制 null-space / 数值 IK 的收敛行为。
    declare_parameter<std::string>("ik_solver_method", "dls");
    declare_parameter<int>("ik_max_iterations", 300);
    declare_parameter<double>("ik_position_tolerance", 1.0e-4);
    declare_parameter<double>("ik_orientation_tolerance", 1.0e-3);
    declare_parameter<double>("ik_damping", 1.0e-4);
    declare_parameter<double>("ik_step_size", 0.15);
    declare_parameter<double>("ik_lm_initial_damping", 1.0e-2);
    declare_parameter<double>("ik_lm_decrease_factor", 0.5);
    declare_parameter<double>("ik_lm_increase_factor", 2.0);
    declare_parameter<bool>("enable_joint_limit_nullspace", true);
    declare_parameter<double>("joint_limit_nullspace_gain", 0.05);

    // 这组参数用于势场法避障示例。
    declare_parameter<int>("max_steps", 500);
    declare_parameter<double>("attractive_position_gain", 1.2);
    declare_parameter<double>("attractive_orientation_gain", 0.8);
    declare_parameter<double>("repulsive_gain", 0.015);
    declare_parameter<double>("influence_distance", 0.20);
    declare_parameter<double>("joint_centering_gain", 0.02);
    declare_parameter<double>("jacobian_damping", 1.0e-4);
    declare_parameter<double>("goal_position_tolerance", 0.01);
    declare_parameter<double>("goal_orientation_tolerance", 0.08);
    declare_parameter<double>("max_joint_step_norm", 0.08);
    declare_parameter<bool>("hold_orientation", true);

    // 这组参数用于 MoveIt 避障规划示例。
    declare_parameter<std::string>("planning_group", "manipulator");
    declare_parameter<std::string>("planning_frame", "world");
    declare_parameter<std::string>("planning_pipeline", "ompl");
    declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
    declare_parameter<double>("planning_time", 5.0);
    declare_parameter<int>("planning_attempts", 5);
    declare_parameter<double>("max_velocity_scaling_factor", 0.2);
    declare_parameter<double>("max_acceleration_scaling_factor", 0.2);

    const std::string urdf_path = get_parameter("urdf_path").as_string();
    const std::string end_effector_frame = get_parameter("end_effector_frame").as_string();

    // 先加载 Pinocchio 机器人模型。
    // 后续无论是 IK、势场还是 MoveIt 对照，都共享这同一个模型入口。
    robot_model_ = std::make_shared<RobotModel>(urdf_path, end_effector_frame);
    if (!robot_model_->loadURDF())
    {
      throw std::runtime_error("Failed to load robot model from URDF: " + urdf_path);
    }

    // `ExperimentExamples` 负责组织实验本身，
    // 当前节点则负责参数注入、结果发布和日志输出。
    examples_ = std::make_unique<ExperimentExamples>(robot_model_);
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 10);

    // 这里构造一个不接管生命周期的别名 shared_ptr，
    // 便于在构造阶段把当前节点句柄传给 MoveIt 相关接口。
    const auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
    continuous_random_goal_loop_ = shouldRunContinuousRandomGoalLoop();
    current_start_configuration_ = loadStartConfiguration();

    // 连续随机目标模式下，不要求构造阶段就必须规划成功；
    // 节点会靠后续重规划定时器持续尝试寻找新的可行目标。
    if (continuous_random_goal_loop_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Continuous random-goal planning is enabled. "
        "A new target will be sampled every %.2f seconds.",
        get_parameter("random_goal_period_sec").as_double());
      tryPlanRandomGoal(node_handle, /* log_searching = */ false);
    }
    else
    {
      ExperimentExampleOutput output;
      if (!runSelectedExperiment(node_handle, output))
      {
        throw std::runtime_error("Failed to generate trajectory for the selected experiment mode.");
      }

      applyPlannedOutput(output, /* update_start_configuration = */ true);
      // 摘要日志由实验模块统一拼好，这里直接打印即可。
      RCLCPP_INFO(get_logger(), "%s", output.summary.c_str());
    }

    // 定时器频率决定 `/joint_states` 的播放节奏。
    // 它并不重新规划，只是把已有离散轨迹点按固定频率送出去。
    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ExperimentExamplesDemoNode::publishNextPoint, this));

    if (continuous_random_goal_loop_)
    {
      const double random_goal_period_sec =
        std::max(0.5, get_parameter("random_goal_period_sec").as_double());
      random_goal_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(random_goal_period_sec)),
        [this, node_handle]() {
          tryPlanRandomGoal(node_handle, /* log_searching = */ true);
        });
    }
  }

private:
  static std::string defaultUrdfPath()
  {
    // 默认直接从 `robot_description` 包里取 URDF，
    // 这样 demo 节点本身不需要写死工作区绝对路径。
    return ament_index_cpp::get_package_share_directory("robot_description") + "/urdf/robot.urdf";
  }

  static double clampDouble(double value, double lower, double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  geometry_msgs::msg::Pose loadTargetPose() const
  {
    const std::string mode = get_parameter("experiment_mode").as_string();
    const bool use_safe_demo_target = get_parameter("use_safe_demo_target").as_bool();
    if (use_safe_demo_target && mode == "nullspace_ik")
    {
      // 对 null-space 对照实验，默认优先使用一组“已知可达”的安全目标。
      // 做法不是手填笛卡尔 pose，而是先构造一个可达关节姿态，再做一次 FK。
      const Eigen::VectorXd neutral = robot_model_->neutralConfiguration();
      Eigen::VectorXd sample = neutral;
      sample[0] = -0.55;
      sample[1] = 0.85;
      sample[2] = 1.10;
      sample[3] = -0.95;
      sample[4] = 1.20;
      sample[5] = 0.80;
      sample[6] = -1.35;
      sample = robot_model_->clampToLimits(sample);

      geometry_msgs::msg::Pose pose;
      if (!robot_model_->forwardKinematics(sample, pose))
      {
        throw std::runtime_error("Failed to derive safe demo target pose from forward kinematics.");
      }
      return pose;
    }

    if (use_safe_demo_target && mode == "potential_field_avoidance")
    {
      // 势场法比纯 IK 更容易受目标姿态和障碍物布局影响而陷入停滞，
      // 因此这里单独给它准备一组更温和、已知可达的目标姿态。
      const Eigen::VectorXd neutral = robot_model_->neutralConfiguration();
      Eigen::VectorXd sample = neutral;
      sample[0] = 0.20;
      sample[1] = 0.30;
      sample[3] = -0.35;
      sample[5] = 0.25;
      sample = robot_model_->clampToLimits(sample);

      geometry_msgs::msg::Pose pose;
      if (!robot_model_->forwardKinematics(sample, pose))
      {
        throw std::runtime_error(
                "Failed to derive safe potential-field target pose from forward kinematics.");
      }
      return pose;
    }

    // 若未启用安全目标，就按 YAML 里的位置和姿态参数直接组装目标 pose。
    const std::vector<double> position = get_parameter("target_position").as_double_array();
    const std::vector<double> orientation = get_parameter("target_orientation").as_double_array();
    if (position.size() != 3 || orientation.size() != 4)
    {
      throw std::runtime_error(
              "target_position must have 3 elements and target_orientation must have 4 elements.");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];
    return pose;
  }

  Eigen::VectorXd loadStartConfiguration() const
  {
    const std::vector<double> values = get_parameter("start_configuration").as_double_array();
    if (values.size() != robot_model_->dof())
    {
      throw std::runtime_error("start_configuration dimension does not match robot dof.");
    }

    Eigen::VectorXd q(static_cast<Eigen::Index>(values.size()));
    for (std::size_t idx = 0; idx < values.size(); ++idx)
    {
      // YAML 里的 `std::vector<double>` 需要逐元素拷贝进 Eigen 向量，
      // 后续 Pinocchio / IK / 轨迹规划都以 Eigen 作为主要数值容器。
      q[static_cast<Eigen::Index>(idx)] = values[idx];
    }

    return robot_model_->clampToLimits(q);
  }

  std::vector<SphericalObstacle> loadObstacles() const
  {
    const std::vector<double> centers = get_parameter("obstacle_centers").as_double_array();
    const std::vector<double> radii = get_parameter("obstacle_radii").as_double_array();
    const std::vector<double> margins =
      get_parameter("obstacle_safety_margins").as_double_array();

    if (centers.size() % 3 != 0 || centers.size() / 3 != radii.size() || radii.size() != margins.size())
    {
      throw std::runtime_error(
              "Obstacle parameters must satisfy: len(obstacle_centers) = 3 * len(obstacle_radii) "
              "and len(obstacle_radii) = len(obstacle_safety_margins).");
    }

    std::vector<SphericalObstacle> obstacles;
    obstacles.reserve(radii.size());
    for (std::size_t idx = 0; idx < radii.size(); ++idx)
    {
      // 每 3 个中心坐标对应一个球形障碍物，
      // 同时把半径和安全边界也装配进统一结构体，便于下游规划器复用。
      SphericalObstacle obstacle;
      obstacle.name = "obstacle_" + std::to_string(idx);
      obstacle.center = Eigen::Vector3d(
        centers[3 * idx], centers[3 * idx + 1], centers[3 * idx + 2]);
      obstacle.radius = radii[idx];
      obstacle.safety_margin = margins[idx];
      obstacles.push_back(obstacle);
    }

    return obstacles;
  }

  ExperimentExampleInput loadCommonInput() const
  {
    // 这一步把分散在参数服务器里的公共字段重新收拢成一个实验输入对象，
    // 这样三类实验都可以复用同一套上层装配逻辑。
    ExperimentExampleInput input;
    input.target_pose = loadTargetPose();
    input.start_configuration = loadStartConfiguration();
    input.obstacles = loadObstacles();
    input.segment_duration = get_parameter("segment_duration").as_double();
    input.sample_period = get_parameter("sample_period").as_double();
    input.cartesian_waypoint_count = get_parameter("cartesian_waypoint_count").as_int();
    return input;
  }

  ExperimentExampleInput loadCommonInputForTarget(const geometry_msgs::msg::Pose & target_pose) const
  {
    ExperimentExampleInput input;
    input.target_pose = target_pose;
    input.start_configuration = current_start_configuration_;
    input.obstacles = loadObstacles();
    input.segment_duration = get_parameter("segment_duration").as_double();
    input.sample_period = get_parameter("sample_period").as_double();
    input.cartesian_waypoint_count = get_parameter("cartesian_waypoint_count").as_int();
    return input;
  }

  IKOptions loadIKOptions() const
  {
    IKOptions options;
    const std::string method = get_parameter("ik_solver_method").as_string();
    // 运行时通过字符串切换数值 IK 方法，便于在 YAML 中快速对照 DLS / LM。
    options.solver_method =
      (method == "lm") ? IKSolverMethod::LevenbergMarquardt : IKSolverMethod::DampedLeastSquares;
    options.max_iterations = get_parameter("ik_max_iterations").as_int();
    options.position_tolerance = get_parameter("ik_position_tolerance").as_double();
    options.orientation_tolerance = get_parameter("ik_orientation_tolerance").as_double();
    options.damping = get_parameter("ik_damping").as_double();
    options.step_size = get_parameter("ik_step_size").as_double();
    options.lm_initial_damping = get_parameter("ik_lm_initial_damping").as_double();
    options.lm_decrease_factor = get_parameter("ik_lm_decrease_factor").as_double();
    options.lm_increase_factor = get_parameter("ik_lm_increase_factor").as_double();
    options.enable_joint_limit_nullspace =
      get_parameter("enable_joint_limit_nullspace").as_bool();
    options.joint_limit_nullspace_gain =
      get_parameter("joint_limit_nullspace_gain").as_double();
    return options;
  }

  ObstacleAvoidanceOptions loadPotentialFieldOptions() const
  {
    ObstacleAvoidanceOptions options;
    // 势场法的参数大多是“增益 + 阈值”类型，
    // 这里集中从参数服务器取出后再整体传给规划器。
    options.max_steps = get_parameter("max_steps").as_int();
    options.sample_period = get_parameter("sample_period").as_double();
    options.attractive_position_gain = get_parameter("attractive_position_gain").as_double();
    options.attractive_orientation_gain = get_parameter("attractive_orientation_gain").as_double();
    options.repulsive_gain = get_parameter("repulsive_gain").as_double();
    options.influence_distance = get_parameter("influence_distance").as_double();
    options.joint_centering_gain = get_parameter("joint_centering_gain").as_double();
    options.jacobian_damping = get_parameter("jacobian_damping").as_double();
    options.goal_position_tolerance = get_parameter("goal_position_tolerance").as_double();
    options.goal_orientation_tolerance = get_parameter("goal_orientation_tolerance").as_double();
    options.max_joint_step_norm = get_parameter("max_joint_step_norm").as_double();
    options.hold_orientation = get_parameter("hold_orientation").as_bool();
    options.collision_frames = get_parameter("collision_frames").as_string_array();
    return options;
  }

  MoveItObstacleAvoidanceOptions loadMoveItOptions() const
  {
    MoveItObstacleAvoidanceOptions options;
    // MoveIt 选项更接近规划管线配置：
    // planning group、pipeline、planner id、容差和速度缩放都在这里汇总。
    options.planning_group = get_parameter("planning_group").as_string();
    options.end_effector_link = get_parameter("end_effector_frame").as_string();
    options.planning_frame = get_parameter("planning_frame").as_string();
    options.planning_pipeline = get_parameter("planning_pipeline").as_string();
    options.planner_id = get_parameter("planner_id").as_string();
    options.planning_time = get_parameter("planning_time").as_double();
    options.planning_attempts = get_parameter("planning_attempts").as_int();
    options.max_velocity_scaling_factor =
      get_parameter("max_velocity_scaling_factor").as_double();
    options.max_acceleration_scaling_factor =
      get_parameter("max_acceleration_scaling_factor").as_double();
    options.goal_position_tolerance = get_parameter("goal_position_tolerance").as_double();
    options.goal_orientation_tolerance = get_parameter("goal_orientation_tolerance").as_double();
    options.hold_orientation = get_parameter("hold_orientation").as_bool();
    return options;
  }

  bool runSelectedExperiment(const rclcpp::Node::SharedPtr & node_handle,
                             ExperimentExampleOutput & output)
  {
    const std::string mode = get_parameter("experiment_mode").as_string();
    // 三类实验都共享同一份公共输入，
    // 差异只体现在后续使用哪种求解 / 规划后端。
    const ExperimentExampleInput input = loadCommonInput();
    return runExperimentWithInput(node_handle, mode, input, output);
  }

  bool runExperimentWithInput(const rclcpp::Node::SharedPtr & node_handle,
                              const std::string & mode,
                              const ExperimentExampleInput & input,
                              ExperimentExampleOutput & output)
  {
    // 这个分发器允许节点既能复用 YAML 中的固定目标，
    // 也能在连续随机模式下临时注入一个新采样的目标 pose。

    if (mode == "nullspace_ik")
    {
      return examples_->runNullspaceIKDemo(input, loadIKOptions(), output);
    }

    if (mode == "potential_field_avoidance")
    {
      return examples_->runPotentialFieldAvoidanceDemo(input, loadPotentialFieldOptions(), output);
    }

    if (mode == "moveit_avoidance")
    {
      return examples_->runMoveItAvoidanceDemo(node_handle, input, loadMoveItOptions(), output);
    }

    RCLCPP_ERROR(
      get_logger(),
      "Unsupported experiment_mode '%s'. Supported values: nullspace_ik, "
      "potential_field_avoidance, moveit_avoidance.",
      mode.c_str());
    return false;
  }

  bool shouldRunContinuousRandomGoalLoop() const
  {
    // 目前连续随机目标模式只对势场法 demo 开放，
    // 因为它最适合做“不断给目标、不断尝试规划”的在线演示。
    return get_parameter("continuous_random_goal_loop").as_bool() &&
           get_parameter("experiment_mode").as_string() == "potential_field_avoidance";
  }

  geometry_msgs::msg::Pose sampleRandomTargetPose() const
  {
    // 为了提高连续规划模式下的收敛率，这里不再做“全关节大范围乱采样”，
    // 而是以当前起始关节姿态为中心，在局部邻域里采样一个新关节目标。
    // 这样得到的末端 pose 往往更接近当前姿态，更适合局部数值方法和势场法收敛。
    const Eigen::VectorXd lower = robot_model_->lowerPositionLimits();
    const Eigen::VectorXd upper = robot_model_->upperPositionLimits();
    const double margin_ratio = clampDouble(
      get_parameter("random_goal_joint_limit_margin_ratio").as_double(), 0.0, 0.45);
    const double local_delta_ratio = clampDouble(
      get_parameter("random_goal_local_joint_delta_ratio").as_double(), 0.02, 0.50);

    Eigen::VectorXd q = current_start_configuration_.size() == static_cast<Eigen::Index>(robot_model_->dof()) ?
      current_start_configuration_ :
      robot_model_->neutralConfiguration();
    for (Eigen::Index idx = 0; idx < q.size(); ++idx)
    {
      const double range = upper[idx] - lower[idx];
      const double margin = 0.5 * margin_ratio * range;
      const double max_delta = 0.5 * local_delta_ratio * range;
      const double sample_lower = std::max(lower[idx] + margin, q[idx] - max_delta);
      const double sample_upper = std::min(upper[idx] - margin, q[idx] + max_delta);
      std::uniform_real_distribution<double> distribution(sample_lower, sample_upper);
      q[idx] = distribution(random_engine_);
    }

    geometry_msgs::msg::Pose pose;
    if (!robot_model_->forwardKinematics(robot_model_->clampToLimits(q), pose))
    {
      throw std::runtime_error("Failed to derive a random target pose from forward kinematics.");
    }
    return pose;
  }

  bool passesRandomGoalScreening(const geometry_msgs::msg::Pose & target_pose) const
  {
    // 这里额外做一次 IK 可达性筛选：
    // 如果数值 IK 以当前起始姿态为初值都很难收敛到这个 pose，
    // 那么直接把它交给连续 demo 往往只会反复失败。
    if (!get_parameter("random_goal_precheck_with_ik").as_bool())
    {
      return true;
    }

    IKOptions screening_options = loadIKOptions();
    screening_options.max_iterations = std::max(screening_options.max_iterations, 400);
    screening_options.enable_joint_limit_nullspace = false;

    IKSolver screening_solver(robot_model_, screening_options);
    return screening_solver.isPoseReachable(target_pose, nullptr, current_start_configuration_);
  }

  void applyPlannedOutput(const ExperimentExampleOutput & output, bool update_start_configuration)
  {
    // 成功规划后就立即替换当前正在播放的轨迹，
    // 这样下一轮发布会无缝切换到新目标对应的轨迹上。
    trajectory_points_ = output.trajectory_points;
    publish_index_ = 0;

    if (update_start_configuration && !output.joint_waypoints.empty())
    {
      current_start_configuration_ = output.joint_waypoints.back();
    }
  }

  void tryPlanRandomGoal(const rclcpp::Node::SharedPtr & node_handle, bool log_searching)
  {
    if (log_searching)
    {
      RCLCPP_INFO(get_logger(), "Searching for a new random target pose...");
    }

    const int attempts = std::max(
      1, static_cast<int>(get_parameter("random_goal_attempts_per_cycle").as_int()));
    const bool reuse_last_success_as_start =
      get_parameter("reuse_last_success_as_start").as_bool();

    for (int attempt = 0; attempt < attempts; ++attempt)
    {
      geometry_msgs::msg::Pose random_target_pose = sampleRandomTargetPose();
      if (!passesRandomGoalScreening(random_target_pose))
      {
        RCLCPP_INFO(
          get_logger(),
          "Random target attempt %d/%d was filtered out by IK precheck. Sampling another one...",
          attempt + 1,
          attempts);
        continue;
      }

      ExperimentExampleInput input = loadCommonInputForTarget(random_target_pose);
      if (!reuse_last_success_as_start)
      {
        input.start_configuration = loadStartConfiguration();
      }

      ExperimentExampleOutput output;
      if (runExperimentWithInput(
          node_handle, "potential_field_avoidance", input, output))
      {
        applyPlannedOutput(output, /* update_start_configuration = */ true);
        RCLCPP_INFO(
          get_logger(),
          "Found a valid random target on attempt %d/%d.\n%s",
          attempt + 1,
          attempts,
          output.summary.c_str());
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Random target attempt %d/%d did not converge. Continuing to search...",
        attempt + 1,
        attempts);
    }

    RCLCPP_WARN(
      get_logger(),
      "No valid random target was found in this replanning cycle. Waiting for the next cycle...");
  }

  void publishNextPoint()
  {
    if (trajectory_points_.empty())
    {
      return;
    }

    // 每次定时器触发都把当前缓存轨迹点转成一条 `JointState`，
    // 这样 RViz / robot_state_publisher / 其他订阅者都能直接消费。
    const JointTrajectoryPoint & point = trajectory_points_[publish_index_];
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = robot_model_->jointNames();
    message.position.assign(point.positions.data(), point.positions.data() + point.positions.size());
    message.velocity.assign(point.velocities.data(), point.velocities.data() + point.velocities.size());
    message.effort.assign(
      point.accelerations.data(), point.accelerations.data() + point.accelerations.size());
    publisher_->publish(message);

    if (publish_index_ + 1 < trajectory_points_.size())
    {
      // 播放到最后一个点后保持停在末端，
      // 这样观察者仍能持续看到最终姿态，而不是回绕到起点。
      ++publish_index_;
    }
  }

  /// 统一的机器人模型入口，供 FK / IK / Jacobian / joint name 查询复用。
  std::shared_ptr<RobotModel> robot_model_;

  /// 组织三类对照实验的上层对象。
  std::unique_ptr<ExperimentExamples> examples_;

  /// 将离散轨迹点发布为 `/joint_states` 的发布器。
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  /// 按固定频率播放轨迹的 ROS2 定时器。
  rclcpp::TimerBase::SharedPtr timer_;

  /// 连续随机目标模式下定时触发重规划的定时器。
  rclcpp::TimerBase::SharedPtr random_goal_timer_;

  /// 当前实验生成出的离散轨迹缓存。
  std::vector<JointTrajectoryPoint> trajectory_points_;

  /// 记录下一个要发布的轨迹点索引。
  std::size_t publish_index_{0};

  /// 连续随机目标模式下，下一轮规划默认从上一次成功终点继续出发。
  Eigen::VectorXd current_start_configuration_;

  /// 是否启用了连续随机目标重规划模式。
  bool continuous_random_goal_loop_{false};

  /// 随机目标采样器，共享一台伪随机引擎避免重复播种。
  mutable std::mt19937 random_engine_{std::random_device{}()};
};
}  // namespace robot_core

int main(int argc, char ** argv)
{
  // main 本身只负责节点生命周期管理；
  // 真正的实验装配、执行和日志输出都在节点构造与回调内部完成。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_core::ExperimentExamplesDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
