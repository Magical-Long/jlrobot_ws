#include "rclcpp/rclcpp.hpp"
#include "robot_ros/gazebo_admittance_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_ros::GazeboAdmittanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
