import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 这个 launch 主要用于最基础的模型可视化：
    # - 读取 URDF；
    # - 启动 robot_state_publisher；
    # - 按需启动 joint_state_publisher(_gui) 和 RViz。
    package_name = "robot_description"

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")

    # 直接把 URDF 文本装进 `robot_description` 参数，
    # 便于 `robot_state_publisher` 立即消费。
    with open(urdf_path, "r") as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {
        "robot_description": robot_description_content
    }

    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description=(
            "Whether to start joint_state_publisher_gui. "
            "If false, normal joint_state_publisher will be started."
        )
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2."
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # 使用 GUI 版本时，可以手动拖动每个关节，快速检查 URDF 关节方向和限位。
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_gui)
    )

    # 非 GUI 版本适合远程环境或不需要手动交互的场景。
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_gui)
    )

    # RViz 仅用于观察，不参与模型或话题生成。
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_gui,
        declare_use_rviz,

        robot_state_publisher_node,

        joint_state_publisher_gui_node,
        joint_state_publisher_node,

        rviz_node,
    ])
