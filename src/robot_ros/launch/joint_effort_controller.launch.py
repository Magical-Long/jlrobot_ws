import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("robot_description")
    ros_share = get_package_share_directory("robot_ros")
    default_urdf_path = os.path.join(description_share, "urdf", "robot.urdf")
    default_config_path = os.path.join(
        ros_share,
        "config",
        "gazebo_effort_controller.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_config",
            default_value=default_config_path,
            description="YAML parameter file for the joint-space PD torque controller.",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=default_urdf_path,
            description="URDF path used by robot_core for joint-space PD torque control.",
        ),
        DeclareLaunchArgument(
            "end_effector_frame",
            default_value="iiwa_link_ee",
            description="End-effector frame name used by the underlying robot model.",
        ),
        Node(
            package="robot_ros",
            executable="gazebo_effort_controller",
            name="joint_effort_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("controller_config"),
                {
                    "urdf_path": LaunchConfiguration("urdf_path"),
                    "end_effector_frame": LaunchConfiguration("end_effector_frame"),
                },
            ],
        ),
    ])
