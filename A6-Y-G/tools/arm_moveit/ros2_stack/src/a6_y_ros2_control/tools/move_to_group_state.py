#!/usr/bin/env python3
"""Move the A6_Y arm slowly to a named SRDF group state."""

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def nonnegative_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return value


def duration_msg(seconds: float) -> Duration:
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanoseconds)


def parse_joint_value(text: str) -> tuple[str, float]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("expected JOINT=VALUE")
    joint, value_text = text.split("=", 1)
    joint = joint.strip()
    if not joint:
        raise argparse.ArgumentTypeError("joint name must not be empty")
    value = float(value_text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("joint value must be finite")
    return joint, value


def default_srdf_path() -> Path:
    return Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "y1_6.srdf"


def load_group_state(srdf_path: Path, group: str, state_name: str) -> Dict[str, float]:
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    for group_state in root.findall("group_state"):
        if group_state.get("name") != state_name or group_state.get("group") != group:
            continue
        target: Dict[str, float] = {}
        for joint in group_state.findall("joint"):
            name = joint.get("name")
            value = joint.get("value")
            if name is None or value is None:
                continue
            target[name] = float(value)
        if not target:
            raise ValueError(f"group_state {state_name!r} for group {group!r} has no joints")
        return target
    raise ValueError(f"could not find group_state {state_name!r} for group {group!r} in {srdf_path}")


class MoveToGroupState(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("a6_y_move_to_group_state")
        self.args = args
        self.current_joint_state = None
        self.create_subscription(JointState, args.joint_state_topic, self._on_joint_state, 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, args.action)

    def _on_joint_state(self, msg: JointState) -> None:
        self.current_joint_state = msg

    def wait_for_current_positions(self, joints: List[str]) -> Dict[str, float]:
        deadline = time.monotonic() + self.args.joint_state_timeout
        missing = set(joints)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is None:
                continue
            positions = {
                name: float(position)
                for name, position in zip(self.current_joint_state.name, self.current_joint_state.position)
            }
            missing = {joint for joint in joints if joint not in positions}
            if not missing:
                return {joint: positions[joint] for joint in joints}
        raise TimeoutError(
            f"no complete {self.args.joint_state_topic} for joints {sorted(missing)} "
            f"within {self.args.joint_state_timeout:.1f} s"
        )

    def send_goal(self, joints: List[str], current: Dict[str, float], target: Dict[str, float]) -> int:
        self.get_logger().info(f"Waiting for action server {self.args.action}")
        if not self.action_client.wait_for_server(timeout_sec=self.args.action_server_timeout):
            self.get_logger().error(f"Action server {self.args.action} is not available")
            return 2

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints

        start_point = JointTrajectoryPoint()
        start_point.positions = [current[joint] for joint in joints]
        start_point.velocities = [0.0] * len(joints)
        start_point.time_from_start = duration_msg(self.args.hold_duration)

        target_point = JointTrajectoryPoint()
        target_point.positions = [target[joint] for joint in joints]
        target_point.velocities = [0.0] * len(joints)
        target_point.time_from_start = duration_msg(self.args.hold_duration + self.args.move_duration)

        goal.trajectory.points = [start_point, target_point]
        goal.goal_time_tolerance = duration_msg(self.args.goal_time_tolerance)
        goal.goal_tolerance = [
            JointTolerance(name=joint, position=self.args.goal_position_tolerance, velocity=0.0, acceleration=0.0)
            for joint in joints
        ]

        self.get_logger().info(
            f"Sending {self.args.group}/{self.args.state} trajectory: hold {self.args.hold_duration:.1f} s, "
            f"move {self.args.move_duration:.1f} s"
        )
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.action_server_timeout)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("No response from trajectory action server")
            return 3
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return 4

        result_future = goal_handle.get_result_async()
        timeout = self.args.hold_duration + self.args.move_duration + self.args.goal_time_tolerance + 5.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error(f"Trajectory did not finish within {timeout:.1f} s")
            return 5

        result = result_wrapper.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Trajectory failed with error_code={result.error_code}: {result.error_string}"
            )
            return 6

        self.get_logger().info(f"Reached {self.args.group}/{self.args.state}; MoveIt can be started.")
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--srdf", type=Path, default=default_srdf_path())
    parser.add_argument("--group", default="arm")
    parser.add_argument("--state", default="home")
    parser.add_argument("--joint", action="append", default=None, help="Joint order for the trajectory.")
    parser.add_argument("--target-joint", action="append", type=parse_joint_value, default=[])
    parser.add_argument("--action", default="/arm_controller/follow_joint_trajectory")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-state-timeout", type=positive_float, default=8.0)
    parser.add_argument("--action-server-timeout", type=positive_float, default=12.0)
    parser.add_argument("--hold-duration", type=nonnegative_float, default=2.0)
    parser.add_argument("--move-duration", type=positive_float, default=20.0)
    parser.add_argument("--goal-time-tolerance", type=nonnegative_float, default=5.0)
    parser.add_argument("--goal-position-tolerance", type=positive_float, default=0.03)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    joints = list(args.joint) if args.joint else list(DEFAULT_JOINTS)
    try:
        target = load_group_state(args.srdf, args.group, args.state)
        target.update(dict(args.target_joint))
        missing_target = [joint for joint in joints if joint not in target]
        if missing_target:
            raise ValueError(f"target is missing joints: {missing_target}")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveToGroupState(args)
    try:
        current = node.wait_for_current_positions(joints)
        return node.send_goal(joints, current, target)
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
