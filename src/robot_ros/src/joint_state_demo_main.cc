#include "robot_ros/joint_state_demo.hpp"

/**
 * @file joint_state_demo_main.cc
 * @brief `robot_ros::JointStateDemoNode` 的 ROS2 进程入口。
 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库，后续节点创建和通信都依赖这一步。
  rclcpp::init(argc, argv);
  // 构造并运行演示节点；`spin()` 会持续处理定时器和话题回调。
  rclcpp::spin(std::make_shared<robot_ros::JointStateDemoNode>());
  // 节点退出后，显式关闭 ROS2 运行时资源。
  rclcpp::shutdown();
  return 0;
}
