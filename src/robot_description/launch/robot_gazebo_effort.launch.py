import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")

    # 复用通用 Gazebo 启动骨架，只切换：
    # - effort 版本 URDF
    # - effort 控制器 YAML
    # - 主控制器名称
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use simulated clock from Gazebo.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Whether to start the Gazebo client GUI.",
        ),
        DeclareLaunchArgument(
            "pause",
            default_value="false",
            description="Whether to start Gazebo in paused mode.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_share, "world", "custom_world.world"),
            description="Gazebo world file path.",
        ),
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value="/robot_description",
            description="Topic that spawn_entity.py reads to get the robot XML description.",
        ),
        DeclareLaunchArgument(
            "entity_name",
            default_value="iiwa7",
            description="Entity name used when spawning the robot in Gazebo.",
        ),
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value="/controller_manager",
            description="Fully-qualified controller_manager node name used by controller spawners.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Initial x position of the robot in the Gazebo world frame.",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position of the robot in the Gazebo world frame.",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Initial z position of the robot in the Gazebo world frame.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "robot_gazebo.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "gui": LaunchConfiguration("gui"),
                "pause": LaunchConfiguration("pause"),
                "world": LaunchConfiguration("world"),
                "urdf_filename": "robot_gazebo_effort.urdf",
                "controllers_yaml_filename": "iiwa7_effort_controllers.yaml",
                "primary_controller_name": "arm_effort_controller",
                "robot_description_topic": LaunchConfiguration("robot_description_topic"),
                "entity_name": LaunchConfiguration("entity_name"),
                "controller_manager_name": LaunchConfiguration("controller_manager_name"),
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
            }.items(),
        )
    ])
