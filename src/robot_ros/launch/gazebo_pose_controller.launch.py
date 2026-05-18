import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_description")
    default_urdf_path = os.path.join(package_share, "urdf", "robot.urdf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=default_urdf_path,
            description="URDF path used by robot_core for IK and trajectory planning.",
        ),
        DeclareLaunchArgument(
            "end_effector_frame",
            default_value="iiwa_link_ee",
            description="End-effector frame name used by the IK solver.",
        ),
        DeclareLaunchArgument(
            "target_pose_topic",
            default_value="/target_pose",
            description="Input target pose topic.",
        ),
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value="/joint_states",
            description="Joint state topic coming from Gazebo and ros2_control.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/arm_controller/joint_trajectory",
            description="Joint trajectory command topic consumed by arm_controller.",
        ),
        DeclareLaunchArgument(
            "segment_duration",
            default_value="3.0",
            description="Execution duration of each point-to-point trajectory segment in seconds.",
        ),
        DeclareLaunchArgument(
            "sample_period",
            default_value="0.02",
            description="Sampling period used when discretizing the joint trajectory.",
        ),
        DeclareLaunchArgument(
            "position_only",
            default_value="true",
            description="If true, keep the current end-effector orientation and only drive position.",
        ),
        DeclareLaunchArgument(
            "ik_max_iterations",
            default_value="800",
            description="Maximum IK iterations.",
        ),
        DeclareLaunchArgument(
            "ik_solver_method",
            default_value="lm",
            description="IK solver method: lm or dls.",
        ),
        DeclareLaunchArgument(
            "ik_position_tolerance",
            default_value="0.005",
            description="IK position tolerance in meters.",
        ),
        DeclareLaunchArgument(
            "ik_orientation_tolerance",
            default_value="0.08",
            description="IK orientation tolerance in radians.",
        ),
        DeclareLaunchArgument(
            "ik_damping",
            default_value="0.001",
            description="IK damping coefficient.",
        ),
        DeclareLaunchArgument(
            "ik_step_size",
            default_value="0.25",
            description="IK integration step size.",
        ),
        DeclareLaunchArgument(
            "enable_joint_limit_nullspace",
            default_value="true",
            description="Whether to enable joint-limit avoidance in IK.",
        ),
        DeclareLaunchArgument(
            "joint_limit_nullspace_gain",
            default_value="0.03",
            description="Joint-limit avoidance gain used by IK.",
        ),
        Node(
            package="robot_ros",
            executable="gazebo_pose_controller",
            name="gazebo_pose_controller",
            output="screen",
            parameters=[{
                "urdf_path": LaunchConfiguration("urdf_path"),
                "end_effector_frame": LaunchConfiguration("end_effector_frame"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "command_topic": LaunchConfiguration("command_topic"),
                "segment_duration": LaunchConfiguration("segment_duration"),
                "sample_period": LaunchConfiguration("sample_period"),
                "position_only": LaunchConfiguration("position_only"),
                "ik_solver_method": LaunchConfiguration("ik_solver_method"),
                "ik_max_iterations": LaunchConfiguration("ik_max_iterations"),
                "ik_position_tolerance": LaunchConfiguration("ik_position_tolerance"),
                "ik_orientation_tolerance": LaunchConfiguration("ik_orientation_tolerance"),
                "ik_damping": LaunchConfiguration("ik_damping"),
                "ik_step_size": LaunchConfiguration("ik_step_size"),
                "enable_joint_limit_nullspace": LaunchConfiguration("enable_joint_limit_nullspace"),
                "joint_limit_nullspace_gain": LaunchConfiguration("joint_limit_nullspace_gain"),
            }],
        ),
    ])
