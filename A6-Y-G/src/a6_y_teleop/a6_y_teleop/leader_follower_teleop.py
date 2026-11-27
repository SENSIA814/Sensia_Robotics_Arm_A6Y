#!/usr/bin/env python3
"""Joint-space leader-follower teleoperation for two A6-Y arms."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class TeleopConfig:
    control_hz: float = 50.0
    leader_joint_state_topic: str = "/leader/joint_states"
    follower_joint_state_topic: str = "/follower/joint_states"
    command_topic: str = "/follower/direct_position_controller/commands"
    command_type: str = "position_array"
    enable_topic: str = "/teleop/enable"
    emergency_stop_topic: str = "/teleop/emergency_stop"
    raw_action_topic: str = "/teleop/raw_action"
    safe_action_topic: str = "/teleop/safe_action"
    status_topic: str = "/teleop/status"
    require_enable: bool = True
    publish_commands: bool = True
    hold_when_disabled: bool = True

    joint_names: List[str] = field(default_factory=lambda: list(DEFAULT_JOINTS))
    leader_joint_names: List[str] = field(default_factory=lambda: list(DEFAULT_JOINTS))
    leader_joint_index: List[int] = field(default_factory=lambda: list(range(6)))
    signs: List[float] = field(default_factory=lambda: [1.0] * 6)
    scales: List[float] = field(default_factory=lambda: [1.0] * 6)
    offsets: List[float] = field(default_factory=lambda: [0.0] * 6)

    min_position: List[float] = field(default_factory=lambda: [-2.62, 0.0, -2.97, -1.55, -0.9, -3.14])
    max_position: List[float] = field(default_factory=lambda: [2.62, 3.14, 0.0, 1.55, 0.9, 3.14])
    max_velocity: List[float] = field(default_factory=lambda: [5.5, 5.5, 5.5, 3.0, 3.0, 3.0])
    max_joint_step: float = 0.35
    limit_margin: List[float] = field(default_factory=lambda: [0.02] * 6)
    # Power teaching opts in; unrelated launch paths retain their supplied behavior.
    step_reference: str = "feedback"

    smoothing_alpha: float = 1.0
    soft_start_duration: float = 0.1
    command_time_from_start: float = 0.03

    prediction_enabled: bool = False
    prediction_lead_time: List[float] = field(default_factory=lambda: [0.12, 0.15, 0.15, 0.0, 0.0, 0.0])
    prediction_max_lead_delta: List[float] = field(default_factory=lambda: [0.35, 0.45, 0.45, 0.0, 0.0, 0.0])
    prediction_velocity_alpha: float = 0.5

    feedback_timeout: float = 0.5
    require_follower_feedback: bool = True
    require_leader_feedback: bool = True
    status_period: float = 0.5


def default_config_file() -> str:
    try:
        return str(Path(get_package_share_directory("a6_y_teleop")) / "config" / "teleop.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "teleop.yaml")


def nested(data: Dict[str, Any], path: str, default: Any) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def as_float_list(value: Any, default: List[float], expected: int, name: str) -> List[float]:
    if value is None:
        result = list(default)
    else:
        result = [float(item) for item in value]
    if len(result) != expected:
        raise ValueError(f"{name} must contain {expected} values, got {len(result)}")
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def as_float_list_or_scalar(value: Any, default: List[float], expected: int, name: str) -> List[float]:
    if value is None:
        result = list(default)
    elif isinstance(value, (int, float, str)):
        result = [float(value)] * expected
    else:
        result = [float(item) for item in value]
    if len(result) != expected:
        raise ValueError(f"{name} must contain {expected} values, got {len(result)}")
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def as_int_list(value: Any, default: List[int], expected: int, name: str) -> List[int]:
    if value is None:
        result = list(default)
    else:
        result = [int(item) for item in value]
    if len(result) != expected:
        raise ValueError(f"{name} must contain {expected} values, got {len(result)}")
    return result


def duration_msg(seconds: float) -> Duration:
    seconds = max(float(seconds), 0.0)
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanoseconds)


def joint_state_positions(msg: JointState) -> Dict[str, float]:
    return {name: float(position) for name, position in zip(msg.name, msg.position)}


def load_config(path: str) -> TeleopConfig:
    config = TeleopConfig()
    config_path = Path(path).expanduser()
    if config_path.is_file():
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    else:
        data = {}

    config.control_hz = float(nested(data, "teleop.control_hz", config.control_hz))
    config.leader_joint_state_topic = str(
        nested(data, "teleop.leader_joint_state_topic", config.leader_joint_state_topic)
    )
    config.follower_joint_state_topic = str(
        nested(data, "teleop.follower_joint_state_topic", config.follower_joint_state_topic)
    )
    config.command_topic = str(nested(data, "teleop.command_topic", config.command_topic))
    config.command_type = str(nested(data, "teleop.command_type", config.command_type)).strip().lower()
    config.enable_topic = str(nested(data, "teleop.enable_topic", config.enable_topic))
    config.emergency_stop_topic = str(nested(data, "teleop.emergency_stop_topic", config.emergency_stop_topic))
    config.raw_action_topic = str(nested(data, "teleop.raw_action_topic", config.raw_action_topic))
    config.safe_action_topic = str(nested(data, "teleop.safe_action_topic", config.safe_action_topic))
    config.status_topic = str(nested(data, "teleop.status_topic", config.status_topic))
    config.require_enable = bool(nested(data, "teleop.require_enable", config.require_enable))
    config.publish_commands = bool(nested(data, "teleop.publish_commands", config.publish_commands))
    config.hold_when_disabled = bool(nested(data, "teleop.hold_when_disabled", config.hold_when_disabled))

    joint_names = nested(data, "joints.names", config.joint_names)
    config.joint_names = [str(name) for name in joint_names]
    expected = len(config.joint_names)
    config.leader_joint_names = [
        str(name) for name in nested(data, "joints.leader_names", list(config.joint_names))
    ]
    config.leader_joint_index = as_int_list(
        nested(data, "joints.leader_joint_index", config.leader_joint_index),
        config.leader_joint_index,
        expected,
        "joints.leader_joint_index",
    )
    config.signs = as_float_list(nested(data, "joints.sign", config.signs), config.signs, expected, "joints.sign")
    config.scales = as_float_list(
        nested(data, "joints.scale", config.scales), config.scales, expected, "joints.scale"
    )
    config.offsets = as_float_list(
        nested(data, "joints.offset", config.offsets), config.offsets, expected, "joints.offset"
    )

    config.min_position = as_float_list(
        nested(data, "limits.min_position", config.min_position),
        config.min_position,
        expected,
        "limits.min_position",
    )
    config.max_position = as_float_list(
        nested(data, "limits.max_position", config.max_position),
        config.max_position,
        expected,
        "limits.max_position",
    )
    config.max_velocity = as_float_list(
        nested(data, "limits.max_velocity", config.max_velocity),
        config.max_velocity,
        expected,
        "limits.max_velocity",
    )
    config.max_joint_step = float(nested(data, "limits.max_joint_step", config.max_joint_step))
    config.step_reference = str(nested(data, "limits.step_reference", config.step_reference))
    config.limit_margin = as_float_list_or_scalar(
        nested(data, "limits.limit_margin", config.limit_margin),
        config.limit_margin,
        expected,
        "limits.limit_margin",
    )

    config.smoothing_alpha = float(nested(data, "smoothing.alpha", config.smoothing_alpha))
    config.soft_start_duration = float(nested(data, "smoothing.soft_start_duration", config.soft_start_duration))
    config.command_time_from_start = float(
        nested(data, "smoothing.command_time_from_start", config.command_time_from_start)
    )

    config.prediction_enabled = bool(nested(data, "prediction.enabled", config.prediction_enabled))
    config.prediction_lead_time = as_float_list(
        nested(data, "prediction.lead_time", config.prediction_lead_time),
        config.prediction_lead_time,
        expected,
        "prediction.lead_time",
    )
    config.prediction_max_lead_delta = as_float_list(
        nested(data, "prediction.max_lead_delta", config.prediction_max_lead_delta),
        config.prediction_max_lead_delta,
        expected,
        "prediction.max_lead_delta",
    )
    config.prediction_velocity_alpha = float(
        nested(data, "prediction.velocity_alpha", config.prediction_velocity_alpha)
    )

    config.feedback_timeout = float(nested(data, "feedback.timeout", config.feedback_timeout))
    config.require_follower_feedback = bool(
        nested(data, "feedback.require_follower_feedback", config.require_follower_feedback)
    )
    config.require_leader_feedback = bool(nested(data, "feedback.require_leader_feedback", config.require_leader_feedback))
    config.status_period = float(nested(data, "feedback.status_period", config.status_period))

    validate_config(config)
    return config


def validate_config(config: TeleopConfig) -> None:
    if config.control_hz <= 0.0 or not math.isfinite(config.control_hz):
        raise ValueError("teleop.control_hz must be finite and positive")
    if not 0.0 < config.smoothing_alpha <= 1.0:
        raise ValueError("smoothing.alpha must be within (0, 1]")
    if config.command_time_from_start <= 0.0:
        raise ValueError("smoothing.command_time_from_start must be positive")
    if config.feedback_timeout <= 0.0:
        raise ValueError("feedback.timeout must be positive")
    if config.command_type not in ("position_array", "trajectory"):
        raise ValueError("teleop.command_type must be 'position_array' or 'trajectory'")
    if config.step_reference not in ("feedback", "previous_command"):
        raise ValueError("limits.step_reference must be 'feedback' or 'previous_command'")
    if config.step_reference == "previous_command":
        if not math.isfinite(config.max_joint_step) or config.max_joint_step <= 0.0:
            raise ValueError("limits.max_joint_step must be finite and positive for previous_command")
        if any(not math.isfinite(v) or v <= 0.0 for v in config.max_velocity):
            raise ValueError("limits.max_velocity must be finite and positive for previous_command")
    if not 0.0 <= config.prediction_velocity_alpha <= 1.0:
        raise ValueError("prediction.velocity_alpha must be within [0, 1]")
    if len(config.leader_joint_names) < max(config.leader_joint_index) + 1:
        raise ValueError("joints.leader_names is shorter than joints.leader_joint_index requires")
    for index, (low, high) in enumerate(zip(config.min_position, config.max_position)):
        if low >= high:
            raise ValueError(f"joint limit {config.joint_names[index]} has min >= max")
    for index, margin in enumerate(config.limit_margin):
        if margin < 0.0:
            raise ValueError(f"limits.limit_margin for {config.joint_names[index]} must be non-negative")
    for index, scale in enumerate(config.scales):
        if scale < 0.0:
            raise ValueError(f"joints.scale for {config.joint_names[index]} must be non-negative")
    for index, lead_time in enumerate(config.prediction_lead_time):
        if lead_time < 0.0:
            raise ValueError(f"prediction.lead_time for {config.joint_names[index]} must be non-negative")
    for index, max_delta in enumerate(config.prediction_max_lead_delta):
        if max_delta < 0.0:
            raise ValueError(f"prediction.max_lead_delta for {config.joint_names[index]} must be non-negative")
    for index, sign in enumerate(config.signs):
        config.signs[index] = 1.0 if sign >= 0.0 else -1.0


class LeaderFollowerTeleop(Node):
    def __init__(self) -> None:
        super().__init__("leader_follower_teleop")
        self.declare_parameter("config_file", default_config_file())
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        self.config = load_config(config_file)

        self.latest_leader: Optional[JointState] = None
        self.latest_follower: Optional[JointState] = None
        self.leader_time = 0.0
        self.follower_time = 0.0
        self.enable = not self.config.require_enable
        self.previous_enable = self.enable
        self.emergency_stop = False
        self.last_safe: Optional[List[float]] = None
        self.last_command_time: Optional[float] = None
        self.soft_start_begin: Optional[List[float]] = None
        self.soft_start_target: Optional[List[float]] = None
        self.soft_start_time = 0.0
        self.previous_raw: Optional[List[float]] = None
        self.previous_raw_time = 0.0
        self.filtered_raw_velocity: Optional[List[float]] = None
        self.last_status_time = 0.0
        self.last_status_text = ""

        self.create_subscription(JointState, self.config.leader_joint_state_topic, self.on_leader_state, 20)
        self.create_subscription(JointState, self.config.follower_joint_state_topic, self.on_follower_state, 20)
        self.create_subscription(Bool, self.config.enable_topic, self.on_enable, 10)
        self.create_subscription(Bool, self.config.emergency_stop_topic, self.on_emergency_stop, 10)

        self.raw_pub = self.create_publisher(JointState, self.config.raw_action_topic, 10)
        self.safe_pub = self.create_publisher(JointState, self.config.safe_action_topic, 10)
        self.status_pub = self.create_publisher(String, self.config.status_topic, 10)
        if self.config.command_type == "position_array":
            self.command_pub = self.create_publisher(Float64MultiArray, self.config.command_topic, 10)
        else:
            self.command_pub = self.create_publisher(JointTrajectory, self.config.command_topic, 10)

        self.timer = self.create_timer(1.0 / self.config.control_hz, self.control_tick)

        self.get_logger().info(
            f"A6-Y teleop ready: leader={self.config.leader_joint_state_topic}, "
            f"follower={self.config.follower_joint_state_topic}, command={self.config.command_topic}, "
            f"command_type={self.config.command_type}, publish_commands={self.config.publish_commands}, "
            f"step_reference={self.config.step_reference}, target_velocity_limits={self.config.max_velocity}, "
            f"soft_start_duration={self.config.soft_start_duration}s"
        )

    def on_leader_state(self, msg: JointState) -> None:
        self.latest_leader = msg
        self.leader_time = time.monotonic()

    def on_follower_state(self, msg: JointState) -> None:
        self.latest_follower = msg
        self.follower_time = time.monotonic()

    def on_enable(self, msg: Bool) -> None:
        self.enable = bool(msg.data)
        if not self.enable:
            self.reset_command_reference()

    def on_emergency_stop(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.reset_command_reference()

    def reset_command_reference(self) -> None:
        if self.config.step_reference == "previous_command":
            self.last_safe = None
            self.last_command_time = None
            self.soft_start_begin = None
            self.soft_start_target = None

    def control_tick(self) -> None:
        now = time.monotonic()
        status_errors = self.feedback_errors(now)
        follower_positions = self.follower_positions()

        if self.emergency_stop:
            self.reset_command_reference()
            self.reset_prediction_state()
            self.publish_status("emergency_stop")
            self.publish_hold_if_possible(follower_positions)
            self.previous_enable = self.enable
            return

        if status_errors:
            self.reset_command_reference()
            self.reset_prediction_state()
            self.publish_status("; ".join(status_errors))
            self.publish_hold_if_possible(follower_positions)
            self.previous_enable = self.enable
            return

        if follower_positions is None:
            self.reset_command_reference()
            self.publish_status("waiting for follower joint state")
            self.previous_enable = self.enable
            return

        if self.config.require_enable and not self.enable:
            self.reset_command_reference()
            self.reset_prediction_state()
            self.last_safe = list(follower_positions)
            self.soft_start_begin = None
            self.soft_start_target = None
            self.publish_status("disabled; holding follower")
            self.publish_hold_if_possible(follower_positions)
            self.previous_enable = self.enable
            return

        raw = self.map_leader_to_follower()
        if raw is None:
            self.reset_command_reference()
            self.reset_prediction_state()
            self.publish_status("waiting for leader joint state")
            self.publish_hold_if_possible(follower_positions)
            self.previous_enable = self.enable
            return
        target = self.apply_prediction(raw, now)

        if (self.enable and not self.previous_enable) or (
                self.config.step_reference == "previous_command" and self.last_command_time is None):
            self.last_safe = list(follower_positions)
            self.soft_start_begin = list(follower_positions)
            self.soft_start_target = list(target)
            self.soft_start_time = now

        candidate = self.apply_soft_start(target, now)
        safe = self.apply_safety(candidate, follower_positions, now=now)

        self.publish_action(self.raw_pub, target)
        self.publish_action(self.safe_pub, safe)
        self.publish_command(safe)
        # This is the last published reference, not a claim of a motor/CAN ACK.
        self.last_safe = list(safe)
        self.last_command_time = now
        self.publish_status("running")
        self.previous_enable = self.enable

    def feedback_errors(self, now: float) -> List[str]:
        errors = []
        if self.config.require_leader_feedback:
            if self.latest_leader is None:
                errors.append("missing leader joint state")
            elif now - self.leader_time > self.config.feedback_timeout:
                errors.append(f"leader feedback timeout {now - self.leader_time:.3f}s")
        if self.config.require_follower_feedback:
            if self.latest_follower is None:
                errors.append("missing follower joint state")
            elif now - self.follower_time > self.config.feedback_timeout:
                errors.append(f"follower feedback timeout {now - self.follower_time:.3f}s")
        return errors

    def follower_positions(self) -> Optional[List[float]]:
        if self.latest_follower is None:
            return None
        by_name = joint_state_positions(self.latest_follower)
        positions = []
        for joint_name in self.config.joint_names:
            if joint_name not in by_name:
                return None
            value = by_name[joint_name]
            if not math.isfinite(value):
                return None
            positions.append(value)
        return positions

    def map_leader_to_follower(self) -> Optional[List[float]]:
        if self.latest_leader is None:
            return None
        by_name = joint_state_positions(self.latest_leader)
        result: List[float] = []
        for follower_index, leader_index in enumerate(self.config.leader_joint_index):
            leader_name = self.config.leader_joint_names[leader_index]
            if leader_name in by_name:
                leader_value = by_name[leader_name]
            elif leader_index < len(self.latest_leader.position):
                leader_value = float(self.latest_leader.position[leader_index])
            else:
                return None
            if not math.isfinite(leader_value):
                return None
            mapped = (
                self.config.signs[follower_index] * self.config.scales[follower_index] * leader_value
                + self.config.offsets[follower_index]
            )
            result.append(mapped)
        return result

    def reset_prediction_state(self) -> None:
        self.previous_raw = None
        self.previous_raw_time = 0.0
        self.filtered_raw_velocity = None

    def apply_prediction(self, raw: List[float], now: float) -> List[float]:
        if not self.config.prediction_enabled:
            return list(raw)

        if self.previous_raw is None:
            self.previous_raw = list(raw)
            self.previous_raw_time = now
            self.filtered_raw_velocity = [0.0] * len(raw)
            return list(raw)

        dt = now - self.previous_raw_time
        if dt <= 1e-6 or dt > self.config.feedback_timeout:
            self.previous_raw = list(raw)
            self.previous_raw_time = now
            self.filtered_raw_velocity = [0.0] * len(raw)
            return list(raw)

        raw_velocity = [
            (value - previous) / dt
            for value, previous in zip(raw, self.previous_raw)
        ]
        if self.filtered_raw_velocity is None:
            self.filtered_raw_velocity = list(raw_velocity)
        else:
            alpha = self.config.prediction_velocity_alpha
            self.filtered_raw_velocity = [
                alpha * velocity + (1.0 - alpha) * previous
                for velocity, previous in zip(raw_velocity, self.filtered_raw_velocity)
            ]

        predicted = []
        for index, value in enumerate(raw):
            max_delta = self.config.prediction_max_lead_delta[index]
            lead_delta = self.filtered_raw_velocity[index] * self.config.prediction_lead_time[index]
            if max_delta > 0.0:
                lead_delta = min(max(lead_delta, -max_delta), max_delta)
            else:
                lead_delta = 0.0
            predicted.append(value + lead_delta)

        self.previous_raw = list(raw)
        self.previous_raw_time = now
        return predicted

    def apply_soft_start(self, raw: List[float], now: float) -> List[float]:
        if self.soft_start_begin is None or self.soft_start_target is None:
            return list(raw)
        duration = max(self.config.soft_start_duration, 1e-6)
        ratio = min(max((now - self.soft_start_time) / duration, 0.0), 1.0)
        eased = ratio * ratio * (3.0 - 2.0 * ratio)
        # Power teaching blends into the live target. A frozen startup endpoint
        # would leave a catch-up step at handoff if the teacher moved meanwhile.
        # The downstream previous-command limiter still bounds every publication.
        target = raw if self.config.step_reference == "previous_command" else self.soft_start_target
        candidate = [
            begin + eased * (goal - begin)
            for begin, goal in zip(self.soft_start_begin, target)
        ]
        if ratio >= 1.0:
            self.soft_start_begin = None
            self.soft_start_target = None
        return candidate

    def apply_safety(self, candidate: List[float], follower_positions: List[float],
                     now: Optional[float] = None) -> List[float]:
        if self.last_safe is None:
            previous = list(follower_positions)
        else:
            previous = self.last_safe

        alpha = self.config.smoothing_alpha
        smoothed = [
            alpha * target + (1.0 - alpha) * old
            for target, old in zip(candidate, previous)
        ]

        safe = []
        period = 1.0 / self.config.control_hz
        dt = period
        command_reference = self.config.step_reference == "previous_command"
        if command_reference:
            if self.last_command_time is None:
                # First publication holds the measured pose; no zero or target jump.
                previous = list(follower_positions)
                dt = 0.0
            else:
                elapsed = (time.monotonic() if now is None else now) - self.last_command_time
                # Never turn a delayed callback into a large catch-up command.
                dt = min(max(elapsed, 0.0), period)
        for index, value in enumerate(smoothed):
            low = self.config.min_position[index] + self.config.limit_margin[index]
            high = self.config.max_position[index] - self.config.limit_margin[index]
            if low > high:
                low = self.config.min_position[index]
                high = self.config.max_position[index]

            limited = min(max(value, low), high)
            max_step = max(self.config.max_joint_step, 0.0)
            if command_reference:
                step_limit = min(max_step, self.config.max_velocity[index] * dt)
                delta = min(max(limited - previous[index], -step_limit), step_limit)
                limited = previous[index] + delta
            else:
                velocity_step = max(self.config.max_velocity[index], 0.0) * period * 1.5
                step_limit = max_step if velocity_step <= 0.0 else min(max_step, velocity_step)
                if step_limit > 0.0:
                    delta = min(max(limited - follower_positions[index], -step_limit), step_limit)
                    limited = follower_positions[index] + delta
            safe.append(min(max(limited, low), high))
        return safe

    def publish_hold_if_possible(self, follower_positions: Optional[List[float]]) -> None:
        if follower_positions is None or not self.config.hold_when_disabled:
            return
        self.publish_action(self.safe_pub, follower_positions)
        self.publish_command(follower_positions)

    def publish_action(self, publisher, positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.config.joint_names)
        msg.position = [float(value) for value in positions]
        publisher.publish(msg)

    def publish_command(self, positions: List[float]) -> None:
        if not self.config.publish_commands:
            return
        if self.config.command_type == "position_array":
            msg = Float64MultiArray()
            msg.data = [float(value) for value in positions]
            self.command_pub.publish(msg)
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.config.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        # Keep velocities empty for streaming teleop; zero velocities make each small target a stop point.
        point.time_from_start = duration_msg(self.config.command_time_from_start)
        msg.points.append(point)
        self.command_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        now = time.monotonic()
        if text == self.last_status_text and now - self.last_status_time < self.config.status_period:
            return
        self.last_status_text = text
        self.last_status_time = now
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main() -> int:
    rclpy.init()
    try:
        node = LeaderFollowerTeleop()
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: failed to start leader_follower_teleop: {exc}")
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
