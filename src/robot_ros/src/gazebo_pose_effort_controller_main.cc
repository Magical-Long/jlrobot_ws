#include "robot_ros/gazebo_pose_effort_controller.hpp"

/**
 * @file gazebo_pose_effort_controller_main.cc
 * @brief `robot_ros::GazeboPoseEffortControllerNode` 的 ROS2 进程入口。
 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 运行时，后续节点创建和话题通信都依赖它。
  rclcpp::init(argc, argv);
  // 运行“位姿目标 -> effort 关节参考”桥接节点。
  rclcpp::spin(std::make_shared<robot_ros::GazeboPoseEffortControllerNode>());
  // 节点退出后显式释放 ROS2 资源。
  rclcpp::shutdown();
  return 0;
}
