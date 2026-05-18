import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_launch_setup(context):
    # 这个 launch 把描述包里的 Gazebo URDF、控制器配置和 Gazebo 本体接起来，
    # 用于最小仿真启动链路验证。
    package_name = "robot_description"
    pkg_share = get_package_share_directory(package_name)
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    urdf_filename = LaunchConfiguration("urdf_filename").perform(context)
    controllers_yaml_filename = LaunchConfiguration(
        "controllers_yaml_filename"
    ).perform(context)
    primary_controller_name_value = LaunchConfiguration(
        "primary_controller_name"
    ).perform(context)

    urdf_path = os.path.join(pkg_share, "urdf", urdf_filename)
    controllers_yaml_path = os.path.join(
        pkg_share, "config", controllers_yaml_filename
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    pause = LaunchConfiguration("pause")
    world = LaunchConfiguration("world")
    robot_description_topic_value = LaunchConfiguration(
        "robot_description_topic"
    ).perform(context)
    entity_name = LaunchConfiguration("entity_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    # 使用全局 `robot_state_publisher` 节点名，
    # 让 ros2_control、joint_state_broadcaster 和 TF 链路都对齐到全局命名空间。
    robot_param_node_name = "/robot_state_publisher"

    # `robot_gazebo.urdf` 里预留了控制器 YAML 路径和参数节点占位符，
    # 这里在 launch 阶段把运行时真实值填进去。
    with open(urdf_path, "r") as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description_content = robot_description_content.replace(
        "__CONTROLLERS_YAML_PATH__",
        controllers_yaml_path,
    )
    robot_description_content = robot_description_content.replace(
        "__ROBOT_PARAM_NODE__",
        robot_param_node_name,
    )

    robot_description = {"robot_description": robot_description_content}

    # 先包含 Gazebo 官方 launch，再把我们的机器人实体插进去。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "pause": pause,
            "world": world,
        }.items(),
    )

    # 在仿真中同样需要 `robot_state_publisher`，负责把关节状态转成 TF。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # `spawn_entity.py` 会从全局 `robot_description` 话题读取模型，
    # 再在 Gazebo 世界中生成实体。
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_iiwa7",
        output="screen",
        arguments=[
            "-entity",
            entity_name,
            "-topic",
            robot_description_topic_value,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
        ],
    )

    # 仅声明 controller_manager 和控制器定义还不够，
    # 还需要显式调用 spawner 把 broadcaster / controller 激活起来。
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    primary_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        name=f"{primary_controller_name_value}_spawner",
        output="screen",
        arguments=[
            primary_controller_name_value,
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    # 等 Gazebo 把实体生成成功后，给 gazebo_ros2_control 留一点初始化时间，
    # 再去激活状态广播器，避免 controller_manager 尚未就绪时过早请求。
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[primary_controller_spawner],
        )
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
    ]


def generate_launch_description():
    package_name = "robot_description"
    pkg_share = get_package_share_directory(package_name)
    default_world_path = os.path.join(pkg_share, "world", "custom_world.world")

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
            default_value=default_world_path,
            description="Gazebo world file path. Defaults to the package custom_world.world.",
        ),
        DeclareLaunchArgument(
            "urdf_filename",
            default_value="robot_gazebo.urdf",
            description="URDF filename under robot_description/urdf used for Gazebo simulation.",
        ),
        DeclareLaunchArgument(
            "controllers_yaml_filename",
            default_value="iiwa7_controllers.yaml",
            description="Controller YAML filename under robot_description/config.",
        ),
        DeclareLaunchArgument(
            "primary_controller_name",
            default_value="arm_controller",
            description="Primary controller to spawn after joint_state_broadcaster.",
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
        OpaqueFunction(function=_build_launch_setup),
    ])
