#!/usr/bin/env python3
"""Publish one episode event for teleoperation recording."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


VALID_EVENTS = ["record", "start", "stop", "success", "failure", "discard", "pause", "resume", "safety_stop"]


class EpisodeMarker(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("a6_y_episode_marker")
        self.pub = self.create_publisher(String, topic, 10)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("event", choices=VALID_EVENTS)
    parser.add_argument("--topic", default="/episode/event")
    parser.add_argument("--repeat", type=int, default=3)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = EpisodeMarker(args.topic)
    msg = String()
    msg.data = args.event
    for _ in range(max(args.repeat, 1)):
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info(f"published episode event {args.event!r} to {args.topic}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
