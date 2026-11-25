#!/usr/bin/env python3
"""Check that leader/follower teleop topics are publishing complete joint states."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class TopicChecker(Node):
    def __init__(self, leader_topic: str, follower_topic: str) -> None:
        super().__init__("a6_y_check_teleop_topics")
        self.leader_msg: Optional[JointState] = None
        self.follower_msg: Optional[JointState] = None
        self.create_subscription(JointState, leader_topic, self.on_leader, 10)
        self.create_subscription(JointState, follower_topic, self.on_follower, 10)

    def on_leader(self, msg: JointState) -> None:
        self.leader_msg = msg

    def on_follower(self, msg: JointState) -> None:
        self.follower_msg = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-topic", default="/leader/joint_states")
    parser.add_argument("--follower-topic", default="/follower/joint_states")
    parser.add_argument("--joint", action="append", default=None)
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def positions_by_name(msg: Optional[JointState]) -> Dict[str, float]:
    if msg is None:
        return {}
    return {name: float(position) for name, position in zip(msg.name, msg.position)}


def missing_joints(msg: Optional[JointState], joints: list[str]) -> list[str]:
    by_name = positions_by_name(msg)
    return [joint for joint in joints if joint not in by_name]


def main() -> int:
    args = parse_args()
    joints = args.joint if args.joint else DEFAULT_JOINTS
    rclpy.init()
    node = TopicChecker(args.leader_topic, args.follower_topic)
    deadline = time.monotonic() + max(args.timeout, 0.1)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.leader_msg is not None and node.follower_msg is not None:
                break
        errors = []
        if node.leader_msg is None:
            errors.append(f"no message on {args.leader_topic}")
        if node.follower_msg is None:
            errors.append(f"no message on {args.follower_topic}")
        leader_missing = missing_joints(node.leader_msg, joints)
        follower_missing = missing_joints(node.follower_msg, joints)
        if leader_missing:
            errors.append("leader missing joints: " + ", ".join(leader_missing))
        if follower_missing:
            errors.append("follower missing joints: " + ", ".join(follower_missing))
        if errors:
            for error in errors:
                print("ERROR: " + error, file=sys.stderr)
            return 1
        print("teleop topic check passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
