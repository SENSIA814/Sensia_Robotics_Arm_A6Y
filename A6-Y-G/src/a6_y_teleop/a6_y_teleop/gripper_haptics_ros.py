"""J7-only bridge: fresh native feedback -> mapper contact cap and teacher torque."""
import json
import math
import time
from control_msgs.msg import DynamicJointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String
from .gripper_haptics import GripperHaptics, GripperHapticConfig

JOINT = 'gripper_finger_joint1'


class GripperHapticBridge:
    def __init__(self, node, config):
        self.node = node
        self.engine = GripperHaptics(GripperHapticConfig(**config['parameters']))
        self.rows = {}
        self.received = {}
        self.started = False
        self.failure = ''
        self.effort = node.create_publisher(Float64MultiArray, config['effort_topic'], 1)
        self.gate = node.create_publisher(Bool, config['gate_topic'], 1)
        self.status = node.create_publisher(String, config['status_topic'], 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for role in ('leader', 'follower'):
            node.create_subscription(DynamicJointState, config[role+'_feedback_topic'],
                lambda msg, r=role: self.feedback(r, msg), qos)

    def feedback(self, role, msg):
        matches = [v for n, v in zip(msg.joint_names, msg.interface_values) if n == JOINT]
        if len(matches) != 1:
            self.rows.pop(role, None)
            return
        values = matches[0]
        self.rows[role] = dict(zip(values.interface_names, values.values))
        self.rows[role]['source_age'] = (self.node.get_clock().now().nanoseconds*1e-9 -
            (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9))
        self.received[role] = time.monotonic()

    def errors(self, now):
        if getattr(self, 'failure', ''):
            return [self.failure]
        errors = []
        for role in ('leader', 'follower'):
            row = self.rows.get(role, {})
            elapsed = now-self.received.get(role, -1e9)
            transport = max(0., row.get('source_age', 0.))
            keys = ('position', 'velocity', 'effort', 'feedback_age', 'position_age',
                    'position_valid', 'has_feedback', 'motor_error', 'fault', 'source_age')
            if (not all(math.isfinite(row.get(k, float('nan'))) for k in keys) or
                not 0 <= elapsed < self.engine.c.timeout or
                not -.001 <= row.get('source_age', -1)+elapsed < self.engine.c.timeout or
                row.get('feedback_age', -1) < 0 or row.get('position_age', -1) < 0 or
                not 0 <= row.get('feedback_age', -1)+transport+elapsed < self.engine.c.timeout or
                not 0 <= row.get('position_age', -1)+transport+elapsed < self.engine.c.timeout or
                row.get('position_valid') != 1 or row.get('has_feedback') != 1 or
                row.get('motor_error') != 1 or row.get('fault') != 0):
                errors.append(role+' gripper haptic feedback invalid/stale/disabled')
        if errors and getattr(self, 'started', False):
            self.failure = '; '.join(errors)
        return errors

    def publish(self, torque, enabled, record):
        # Gravity controller independently enforces the same cap and 80 ms timeout.
        self.effort.publish(Float64MultiArray(data=[0.]*6+[torque/self.engine.c.leader_scale]))
        self.gate.publish(Bool(data=enabled))
        self.status.publish(String(data=json.dumps(record, allow_nan=False)))

    def stop(self):
        self.engine.reset()
        self.publish(0., False, dict(state='inactive', teacher_torque=0., contact=False))

    def apply(self, raw, safe, previous, now, starting):
        if starting:
            self.stop()
            return safe
        errors = self.errors(now)
        if errors:
            self.stop()
            raise ValueError('; '.join(errors))
        self.started = True
        row = self.rows['follower']
        torque = row['effort']*self.engine.c.follower_scale
        result = self.engine.step(now, raw[6], previous[6], row['position'], row['velocity'], torque,
                                  leader_velocity=self.rows['leader']['velocity'])
        if result['state'] == 'inactive':
            self.failure = 'gripper haptic control period exceeded timeout'
            self.stop()
            raise ValueError(self.failure)
        output = list(safe)
        if result['contact']:
            # Withdraw accumulated pursuit immediately. No additional closure
            # is allowed while the operator squeezes past the contact point.
            output[6] = min(safe[6], result['target'])
        elif self.engine.rebase is not None:
            # Rebased release still uses the accepted 100 Hz velocity limiter.
            candidate = list(raw)
            candidate[6] = result['target']
            output[6] = self.node.apply_safety(candidate, self.node.follower_positions(), now=now)[6]
        result.update(raw=raw[6], target=output[6], actual=row['position'],
                      follower_torque=torque, wall_time=time.time(),
                      leader_velocity=self.rows['leader']['velocity'], follower_velocity=row['velocity'])
        self.publish(result['teacher_torque'], True, result)
        return output
