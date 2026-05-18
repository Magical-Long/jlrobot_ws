import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 可视化链路依赖 `robot_description` 提供 URDF，
    # 再由 `robot_state_publisher` 把 `/joint_states` 展开成 TF。
    robot_description_share = get_package_share_directory("robot_description")
    urdf_path = os.path.join(robot_description_share, "urdf", "robot.urdf")

    # 这里直接把 URDF 文本读成 `robot_description` 参数，
    # 这样下游节点无需再各自处理文件路径。
    with open(urdf_path, "r") as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {"robot_description": robot_description_content}

    use_rviz = LaunchConfiguration("use_rviz")
    loop_trajectory = LaunchConfiguration("loop_trajectory")
    sample_period = LaunchConfiguration("sample_period")
    segment_duration = LaunchConfiguration("segment_duration")
    demo_start_delay = LaunchConfiguration("demo_start_delay")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz2 alongside the demo publisher.",
        ),
        DeclareLaunchArgument(
            "loop_trajectory",
            default_value="true",
            description="Whether the demo trajectory should repeat forever.",
        ),
        DeclareLaunchArgument(
            "sample_period",
            default_value="0.05",
            description="Sampling period in seconds for the published joint trajectory.",
        ),
        DeclareLaunchArgument(
            "segment_duration",
            default_value="2.0",
            description="Duration in seconds of each waypoint-to-waypoint segment.",
        ),
        DeclareLaunchArgument(
            "demo_start_delay",
            default_value="4.0",
            description="Delay in seconds before starting the joint state demo publisher.",
        ),
        Node(
            # 负责把 URDF 和 `joint_states` 组合成整棵 TF 树。
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            # 这个 demo 节点会周期性发布关节轨迹，
            # 方便在 RViz 中直接观察 `robot_core` 相关模块的输出效果。
            package="robot_ros",
            executable="joint_state_demo",
            name="joint_state_demo",
            output="screen",
            parameters=[{
                "urdf_path": urdf_path,
                "loop_trajectory": loop_trajectory,
                "sample_period": sample_period,
                "segment_duration": segment_duration,
                "demo_start_delay": demo_start_delay,
            }],
        ),
        Node(
            # RViz 只作为可选观察层，不影响 demo 节点本身运行。
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
