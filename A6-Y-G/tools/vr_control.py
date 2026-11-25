#!/usr/bin/env python3
"""Local VR commands. prepare enables the selected controller; disarm removes output."""
import argparse
import json
import os
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from a6_y_teleop_msgs.msg import VrState, VrControl, VrJointReference

NAMES = ('DISABLED', 'HOLD', 'ARMED_HOLD', 'TELEOP', 'STOPPING', 'PAUSED', 'FAULT')
PREFIX = '/arm6/a6_y_joint_impedance_controller'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=('impedance', 'position'), default='impedance')
    parser.add_argument('action', nargs='?', default='status', choices=('status', 'prepare', 'arm', 'pause', 'disarm'))
    args = parser.parse_args()
    prefix = '/arm6/a6_y_vr_position_controller' if args.controller == 'position' else PREFIX
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') in (None, '84'):
        parser.error('set explicit hardware domain (mock=86) and ROS_LOCALHOST_ONLY=1')
    rclpy.init(); node = rclpy.create_node('vr_local_operator')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latest = {}
    node.create_subscription(VrState, prefix+'/vr_state', lambda v: latest.update(state=v), qos)
    node.create_subscription(VrJointReference, prefix+'/vr_reference', lambda v: latest.update(reference=v), qos)
    pub = node.create_publisher(VrControl, prefix+'/vr_control', 10)
    def wait(predicate, timeout=3.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate(): return True
        return False
    try:
        if not wait(lambda: 'state' in latest and 0 <= time.monotonic_ns()-latest['state'].monotonic_ns < 100_000_000):
            raise RuntimeError('no fresh controller state')
        state = latest['state']
        if state.position_control != (args.controller == 'position'):
            raise RuntimeError('controller backend mismatch')
        if args.action == 'status':
            print(json.dumps(dict(controller=args.controller, state=NAMES[state.state], reason=state.reason, prepared=state.prepared,
                gripper_control=state.gripper_control,gripper_reference=state.gripper_reference_position,
                prepare_ramp_complete=state.prepare_ramp_complete,
                output_permitted=state.output_permitted, follow_permitted=state.follow_permitted,
                auto_rearm_after_release=state.auto_rearm_after_release,
                session_reference_travel_limit=state.session_reference_travel_limit,
                session_actual_travel_limit=state.session_actual_travel_limit, feedback_valid=state.feedback_valid,
                controller_epoch=state.controller_epoch, actual=list(state.actual_positions),
                reference=list(state.reference_positions)), indent=2)); return 0
        action = getattr(VrControl, args.action.upper())
        command = VrControl(action=action, controller_epoch=state.controller_epoch)
        if args.action == 'arm':
            if not state.follow_permitted:
                raise RuntimeError('controller disallows ARM in this mode (hold-only commissioning or readonly)')
            if not wait(lambda: 'reference' in latest and latest['reference'].valid and
                        not latest['reference'].squeeze and
                        0 <= time.monotonic_ns()-latest['reference'].generated_monotonic_ns < 50_000_000):
                raise RuntimeError('arm requires fresh valid real input and a released grip')
            ref = latest['reference']; command.producer_id = ref.producer_id; command.session_id = ref.session_id
            command.clutch_epoch = ref.clutch_epoch
        if not wait(lambda: pub.get_subscription_count() > 0): raise RuntimeError('control subscriber unavailable')
        command.request_id = command.generated_monotonic_ns = time.monotonic_ns()
        pub.publish(command)
        if not wait(lambda: latest['state'].request_id == command.request_id):
            raise RuntimeError('no update-loop acknowledgement; inspect status, do not assume success')
        state = latest['state']
        print(f'{args.action}: accepted={state.request_accepted}, state={NAMES[state.state]}, reason={state.reason}')
        return 0 if state.request_accepted else 2
    except RuntimeError as error:
        print(str(error)); return 2
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': raise SystemExit(main())
