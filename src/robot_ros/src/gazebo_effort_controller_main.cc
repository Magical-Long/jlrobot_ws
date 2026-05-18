#include "robot_ros/gazebo_effort_controller.hpp"

/**
 * @file gazebo_effort_controller_main.cc
 * @brief `robot_ros::GazeboEffortControllerNode` 的 ROS2 进程入口。
 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库，后续节点创建和话题通信都依赖这一步。
  rclcpp::init(argc, argv);
  // 运行 Gazebo effort 模式下的关节空间 PD 力矩控制节点。
  rclcpp::spin(std::make_shared<robot_ros::GazeboEffortControllerNode>());
  // 节点退出后显式关闭 ROS2 运行时。
  rclcpp::shutdown();
  return 0;
}
