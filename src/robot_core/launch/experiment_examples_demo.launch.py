import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # `robot_core` 提供实验节点本体与默认 YAML，
    # `robot_description` 提供用于加载 Pinocchio 模型的 URDF。
    robot_core_share = get_package_share_directory("robot_core")
    robot_description_share = get_package_share_directory("robot_description")

    # 默认情况下直接使用安装后的标准资源路径，
    # 这样 launch 文件不会依赖开发机上的工作区绝对路径。
    default_urdf_path = os.path.join(robot_description_share, "urdf", "robot.urdf")
    default_config_path = os.path.join(
        robot_core_share, "config", "experiment_examples_demo.yaml"
    )

    # 通过 LaunchConfiguration 暴露可覆盖入口，
    # 便于用户在命令行上替换 URDF 或整套实验参数文件。
    urdf_path = LaunchConfiguration("urdf_path")
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=default_urdf_path,
                description="URDF file used by the experiment examples demo.",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_path,
                description="YAML parameter file for the experiment examples demo.",
            ),
            Node(
                # 这里真正拉起统一实验节点：
                # - `config_path` 先整体注入 YAML；
                # - `urdf_path` 再单独覆盖进参数服务器。
                package="robot_core",
                executable="experiment_examples_demo_node",
                name="experiment_examples_demo",
                output="screen",
                parameters=[
                    config_path,
                    {"urdf_path": urdf_path},
                ],
            ),
        ]
    )
