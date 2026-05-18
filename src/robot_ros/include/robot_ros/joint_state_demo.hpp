#pragma once

/**
 * @file joint_state_demo.hpp
 * @brief 定义用于 RViz 可视化验证的关节状态发布节点接口。
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_core/robot_model.hpp"
#include "robot_core/trajectory_planner.hpp"
#include "robot_core/ik_solver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_ros
{
  /**
   * @brief 用于 RViz 验证的关节状态发布节点。
   *
   * 这个节点的目标不是控制真实机器人，而是给 `robot_core` 提供一个
   * 最小闭环验证环境：
   * 1. 读取 URDF 并构建 `RobotModel`；
   * 2. 生成一条简单的关节空间示教轨迹；
   * 3. 周期性发布 `/joint_states`；
   * 4. 让 `robot_state_publisher + RViz` 直接显示机械臂运动结果。
   *
   * 当你修改正运动学、逆运动学或轨迹规划后，可以先跑这个节点，
   * 快速观察关节顺序、限位和轨迹连续性是否正常。
   */
  class JointStateDemoNode : public rclcpp::Node
  {
  public:
    /// @brief 构造并初始化整个可视化验证节点。
    JointStateDemoNode();

  private:
    /// @brief 声明并读取 ROS 参数。
    void declareParameters();

    /// @brief 根据参数解析 URDF 路径并加载 `RobotModel`。
    void initializeRobotModel();

    /// @brief 构造一组用于演示的关节路径点。
    /// 这些路径点主要用于快速检查关节顺序、限位和轨迹插值是否合理。
    std::vector<Eigen::VectorXd> createDemoWaypoints() const;

    /// @brief 构造一组用于演示的笛卡尔空间路径点。
    /// 它们会先经过 IK 转成关节路径，再交给轨迹规划器离散。
    std::vector<geometry_msgs::msg::Pose> createDemoCartesianWaypoints() const;

    /// @brief 检查单个位姿是否在当前机械臂的可达工作空间内。
    /// 若调用方提供 `solution`，该接口也会把找到的逆解一并返回出来。
    bool isCartesianWaypointReachable(const geometry_msgs::msg::Pose & pose,
                                      Eigen::VectorXd * solution = nullptr,
                                      const Eigen::VectorXd & initial_guess = Eigen::VectorXd());

    /// @brief 调用 `JointTrajectoryPlanner` 生成完整离散轨迹。
    void buildTrajectory();

    /// @brief 定时发布下一个轨迹点到 `/joint_states`。
    void publishNextState();

    /// @brief 发布当前静止姿态，用于在播放开始前先建立 TF 树。
    void publishCurrentState(std::size_t trajectory_index);

    /// @brief 解析默认 URDF 路径，默认使用 `robot_description` 包中的 `robot.urdf`。
    /// 这样 launch 不传 `urdf_path` 时，节点仍然能单独运行起来。
    std::string resolveDefaultUrdfPath() const;

    /// @brief 将规划轨迹发布到 RViz 消费的 `JointState` 话题。
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    /// @brief 周期触发发布逻辑的定时器。
    rclcpp::TimerBase::SharedPtr publish_timer_;

    /// @brief 底层机器人模型封装。
    std::shared_ptr<robot_core::RobotModel> robot_model_;

    /// @brief 关节空间轨迹规划器，负责把路径点离散成可发布序列。
    std::unique_ptr<robot_core::JointTrajectoryPlanner> trajectory_planner_;

    /// @brief 逆运动学求解器，负责把笛卡尔路径点转换成关节路径点。
    std::unique_ptr<robot_core::IKSolver> ik_solver_;

    /// @brief 当前准备发布的整条离散轨迹。
    std::vector<robot_core::JointTrajectoryPoint> trajectory_;

    /// @brief 当前发布到轨迹中的哪个采样点。
    std::size_t trajectory_index_{0U};

    /// @brief 轨迹播完后是否自动回到开头循环播放。
    bool loop_trajectory_{true};

    /// @brief 轨迹每段持续时间，单位为秒。
    double segment_duration_{2.0};

    /// @brief 轨迹采样周期，单位为秒。
    double sample_period_{0.05};

    /// @brief 正式开始推进轨迹前的等待时间，单位为秒。
    double demo_start_delay_{0.0};

    /// @brief 节点启动时的单调时钟时间点，用于判断何时开始播放轨迹。
    std::chrono::steady_clock::time_point startup_time_;

    /// @brief 节点加载模型时使用的 URDF 路径。
    std::string urdf_path_;

    /// @brief 机器人末端 frame 名称，传给 `RobotModel` 保持模型配置一致。
    std::string end_effector_frame_;
  };
} // namespace robot_ros
