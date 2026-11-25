#!/usr/bin/env python3
"""Exercise real ROS/RSP and mimic TF with synthetic angles, no serial access."""
import json
import math
from pathlib import Path
import tempfile
import time
import uuid
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from model import JOINTS, load_model
from ros_display import RosDisplay


def main():
    namespace = '/a6y_verify/s' + uuid.uuid4().hex[:10]
    context = Context()
    rclpy.init(args=[], context=context)
    observer = rclpy.create_node('verify_observer', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    states, descriptions, markers = [], [], []
    transforms = {}
    def on_tf(message):
        transforms.update({t.child_frame_id: t for t in message.transforms})
    observer.create_subscription(JointState, namespace + '/joint_states', states.append, 10)
    observer.create_subscription(String, namespace + '/robot_description', descriptions.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    observer.create_subscription(TFMessage, namespace + '/tf', on_tf, 10)
    observer.create_subscription(TFMessage, namespace + '/tf_static', on_tf,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    observer.create_subscription(MarkerArray, namespace + '/markers', markers.append, 10)
    model = load_model('motion_left')
    q = [-.4, .5, .6, .1, -.2, .3, -.012]
    try:
        with tempfile.TemporaryDirectory(prefix='a6y-ros-verify-') as temp, patch('scanner.StatusOnlyControl.open', side_effect=AssertionError('hardware forbidden')):
            with RosDisplay(model, temp, namespace, with_rviz=False) as display:
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    display.publish(q, 'SYNTHETIC ROS TEST')
                    executor.spin_once(timeout_sec=.05)
                    if (len(states) >= 5 and descriptions and markers and len(transforms) == 9
                            and abs(transforms['gripper_finger_link2'].transform.translation.y - .0153224) < 1e-8):
                        break
                assert states and list(states[-1].name) == list(JOINTS), 'joint_states not received'
                assert list(states[-1].position) == q
                assert not states[-1].velocity and not states[-1].effort
                assert descriptions and 'file:///' in descriptions[-1].data
                assert len(transforms) == 9, list(transforms)
                assert abs(transforms['link1'].transform.rotation.z - math.sin(.2)) < 1e-8
                assert abs(transforms['gripper_finger_link1'].transform.translation.y + .0153141) < 1e-8
                assert abs(transforms['gripper_finger_link2'].transform.translation.y - .0153224) < 1e-8
                assert 'tool_tcp' in transforms
                assert markers and len(markers[-1].markers) == 8
                # Drain already queued frames, then prove loss doesn't republish q=0.
                until = time.monotonic() + .3
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.02)
                before = len(states)
                until = time.monotonic() + .5
                while time.monotonic() < until:
                    display.publish(None, 'FEEDBACK LOST / POSE FROZEN', True)
                    executor.spin_once(timeout_sec=.02)
                assert len(states) == before, 'stale joint_states republished'
                assert 'FEEDBACK LOST' in markers[-1].markers[0].text
                assert markers[-1].markers[0].color.r == 1.
                topics = [name for name, _ in observer.get_topic_names_and_types() if name.startswith(namespace)]
                assert not any('command' in name for name in topics), topics
                result = {'ok': True, 'hardware_connected': False, 'namespace': namespace,
                          'joint_states_received': len(states), 'tf_frames': sorted(transforms),
                          'mimic_verified': True, 'loss_no_zero_fill_verified': True,
                          'topics': topics, 'rviz_render_verified': False}
            assert all(child.poll() is not None for _, child in display.children)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        observer.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    main()
