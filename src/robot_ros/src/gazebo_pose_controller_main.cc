#include "robot_ros/gazebo_pose_controller.hpp"

/**
 * @file gazebo_pose_controller_main.cc
 * @brief `robot_ros::GazeboPoseControllerNode` 的 ROS2 进程入口。
 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库，后续节点构造和话题通信都依赖这一步。
  rclcpp::init(argc, argv);
  // 运行目标位姿到 Gazebo 轨迹命令的桥接节点。
  rclcpp::spin(std::make_shared<robot_ros::GazeboPoseControllerNode>());
  // 节点退出后关闭 ROS2 运行时。
  rclcpp::shutdown();
  return 0;
}
