#!/usr/bin/env python3

"""
@file keyboard_command_sender.py
@brief 提供一个键盘交互式 ROS2 指令发送工具。

这个脚本面向当前工作空间里的两条上层输入链路：
1. `/target_pose`：发送末端目标位姿，供 pose controller / pose effort controller 使用。
2. `/desired_joint_states`：发送目标关节角，供 joint effort controller 使用。

用户不需要手工拼 ROS 消息，只需要在终端里输入数组即可：
- 位姿模式：`[x, y, z]` 或 `[x, y, z, qx, qy, qz, qw]`
- 关节模式：`[q1, q2, ..., qN]`
"""

import ast
import sys
import threading
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KeyboardCommandSenderNode(Node):
    """键盘交互式指令发送节点。"""

    def __init__(self) -> None:
        super().__init__("keyboard_command_sender")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/target_pose"
        ).value
        self.joint_topic = self.declare_parameter(
            "joint_topic", "/desired_joint_states"
        ).value
        self.joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/joint_states"
        ).value
        self.default_pose_orientation = list(
            self.declare_parameter(
                "default_pose_orientation", [0.0, 0.0, 0.0, 1.0]
            ).value
        )
        self.joint_names = list(self.declare_parameter("joint_names", []).value)

        self.pose_publisher = self.create_publisher(Pose, self.pose_topic, 10)
        self.joint_publisher = self.create_publisher(JointState, self.joint_topic, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.handle_joint_state,
            10,
        )

        self.cached_joint_names: List[str] = []
        if self.joint_names:
            self.cached_joint_names = list(self.joint_names)

        self.get_logger().info(
            "Keyboard command sender is ready. pose_topic=%s, joint_topic=%s, joint_state_topic=%s"
            % (self.pose_topic, self.joint_topic, self.joint_state_topic)
        )

    def handle_joint_state(self, msg: JointState) -> None:
        """缓存最新关节名顺序，便于直接发布 JointState 目标。"""
        if msg.name:
            self.cached_joint_names = list(msg.name)

    def resolve_joint_names(self, expected_size: int) -> List[str]:
        """优先使用缓存到的关节名；若数量不匹配则回退到参数。"""
        if self.cached_joint_names and len(self.cached_joint_names) == expected_size:
            return list(self.cached_joint_names)

        if self.joint_names and len(self.joint_names) == expected_size:
            return list(self.joint_names)

        return []

    def publish_pose_command(self, values: Sequence[float]) -> bool:
        """发布目标位姿。"""
        if len(values) not in (3, 7):
            self.get_logger().error(
                "Pose command must contain 3 or 7 numbers: [x, y, z] or [x, y, z, qx, qy, qz, qw]."
            )
            return False

        pose = Pose()
        pose.position.x = float(values[0])
        pose.position.y = float(values[1])
        pose.position.z = float(values[2])

        orientation = values[3:] if len(values) == 7 else self.default_pose_orientation
        if len(orientation) != 4:
            self.get_logger().error(
                "Default pose orientation must contain exactly 4 numbers."
            )
            return False

        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])

        self.pose_publisher.publish(pose)
        self.get_logger().info(
            "Published target pose: position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]"
            % (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        return True

    def publish_joint_command(self, values: Sequence[float]) -> bool:
        """发布目标关节角。"""
        if not values:
            self.get_logger().error("Joint command must not be empty.")
            return False

        joint_names = self.resolve_joint_names(len(values))
        if not joint_names:
            self.get_logger().error(
                "No matching joint names are available yet. Start the state publisher/controller first, "
                "or pass parameter 'joint_names'."
            )
            return False

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = joint_names
        message.position = [float(value) for value in values]
        message.velocity = [0.0] * len(values)
        message.effort = []

        self.joint_publisher.publish(message)
        self.get_logger().info(
            "Published desired joint positions: %s" % message.position
        )
        return True


def parse_numeric_array(text: str) -> Optional[List[float]]:
    """把用户输入解析成数字数组。"""
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return None

    if not isinstance(value, (list, tuple)):
        return None

    parsed: List[float] = []
    for item in value:
        if not isinstance(item, (int, float)):
            return None
        parsed.append(float(item))
    return parsed


def print_help() -> None:
    """打印交互帮助。"""
    print(
        "\nKeyboard Command Sender\n"
        "Commands:\n"
        "  help                Show this help message\n"
        "  mode pose           Switch to pose mode\n"
        "  mode joint          Switch to joint mode\n"
        "  pose [x,y,z]        Publish target pose with default orientation\n"
        "  pose [x,y,z,qx,qy,qz,qw]\n"
        "                      Publish target pose with full quaternion\n"
        "  joint [q1,...,qN]   Publish desired joint positions\n"
        "  [ ... ]             Publish directly using current mode\n"
        "  quit                Exit the tool\n"
    )


def interactive_loop(node: KeyboardCommandSenderNode) -> int:
    """运行键盘交互主循环。"""
    current_mode = "pose"
    print_help()
    print("Current mode: pose")

    while rclpy.ok():
        try:
            user_input = input(f"{current_mode}> ").strip()
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print()
            return 0

        if not user_input:
            continue

        if user_input in ("quit", "exit"):
            return 0

        if user_input == "help":
            print_help()
            continue

        if user_input == "mode pose":
            current_mode = "pose"
            print("Switched to pose mode.")
            continue

        if user_input == "mode joint":
            current_mode = "joint"
            print("Switched to joint mode.")
            continue

        payload_text = user_input
        explicit_mode: Optional[str] = None

        if user_input.startswith("pose "):
            explicit_mode = "pose"
            payload_text = user_input[len("pose ") :].strip()
        elif user_input.startswith("joint "):
            explicit_mode = "joint"
            payload_text = user_input[len("joint ") :].strip()

        values = parse_numeric_array(payload_text)
        if values is None:
            print("Input must be a numeric array, for example: [0.4, 0.0, 0.6]")
            continue

        target_mode = explicit_mode or current_mode
        if target_mode == "pose":
            node.publish_pose_command(values)
        else:
            node.publish_joint_command(values)

    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """程序入口。"""
    rclpy.init(args=argv)
    node = KeyboardCommandSenderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        return interactive_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
