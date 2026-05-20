#!/usr/bin/env python3

"""
@file ee_force_tester.py
@brief 向 Gazebo 末端虚拟 link 持续发送测试 wrench。
"""

import argparse
import sys
import time
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node


class EeForceTesterNode(Node):
    """持续发布末端测试 wrench 的小工具。"""

    def __init__(self, topic_name: str, rate_hz: float) -> None:
        super().__init__("ee_force_tester")
        self.publisher = self.create_publisher(Wrench, topic_name, 10)
        self.topic_name = topic_name
        self.rate_hz = max(rate_hz, 1.0)

    def publish_wrench(self, values: Sequence[float]) -> None:
        message = Wrench()
        message.force.x = float(values[0])
        message.force.y = float(values[1])
        message.force.z = float(values[2])
        message.torque.x = float(values[3])
        message.torque.y = float(values[4])
        message.torque.z = float(values[5])
        self.publisher.publish(message)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish a constant wrench to the Gazebo ee_link force plugin."
    )
    parser.add_argument("--topic", default="/ee_force_command", help="Wrench command topic.")
    parser.add_argument("--fx", type=float, default=0.0, help="Force x in N.")
    parser.add_argument("--fy", type=float, default=0.0, help="Force y in N.")
    parser.add_argument("--fz", type=float, default=0.0, help="Force z in N.")
    parser.add_argument("--tx", type=float, default=0.0, help="Torque x in Nm.")
    parser.add_argument("--ty", type=float, default=0.0, help="Torque y in Nm.")
    parser.add_argument("--tz", type=float, default=0.0, help="Torque z in Nm.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate in Hz.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="How long to keep publishing in seconds. 0 means until Ctrl+C.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = EeForceTesterNode(args.topic, args.rate)

    values = [args.fx, args.fy, args.fz, args.tx, args.ty, args.tz]
    period = 1.0 / max(args.rate, 1.0)
    start_time = time.monotonic()

    node.get_logger().info(
        "Publishing ee wrench to %s: force=[%.3f, %.3f, %.3f] torque=[%.3f, %.3f, %.3f], rate=%.1f Hz"
        % (args.topic, args.fx, args.fy, args.fz, args.tx, args.ty, args.tz, args.rate)
    )

    try:
        while rclpy.ok():
            node.publish_wrench(values)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

            if args.duration > 0.0 and (time.monotonic() - start_time) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一个全零 wrench，避免 Gazebo 继续保持上一帧外力。
        node.publish_wrench([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
