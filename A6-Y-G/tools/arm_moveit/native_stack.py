"""Launch the archived C++ driver and standard ROS controllers, never Python CAN."""
from copy import deepcopy
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import yaml

from motion_model import JOINTS
from native_model import write_controller_config, write_moveit_config
from ros_display import rviz_config


class NativeFeedback:
    def __init__(self, model, mock):
        self.model, self.mock = model, mock
        self.lock = threading.Lock()
        self.cancel_generation = 0
        self.state = dict(q=None, velocity=None, sample_time=None, diagnostic_time=None, ros_stamp=None,
                          rows={}, status='等待 ros2_control 反馈',
                          error=None, enabled=False, executing=False, completed=0,
                          controller_backend='joint_trajectory_controller', release_confirmed=True, control_ready=False)

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def generation(self):
        with self.lock:
            return self.cancel_generation

    def cancel_motion(self, **_):
        with self.lock:
            self.cancel_generation += 1

    def measured(self):
        state = self.snapshot()
        if state['error'] or state['q'] is None or time.monotonic() - (state['sample_time'] or 0) > .5:
            raise ValueError('ros2_control 反馈缺失、过期或驱动报告故障')
        if not self.mock:
            for name in JOINTS:
                row = state['rows'].get(name, {})
                if row.get('has_feedback', 0) != 1 or row.get('feedback_age', float('inf')) > .5 or row.get('fault', 1) != 0:
                    raise ValueError(f'{name} 的电机反馈缺失、超时或故障')
        return np.asarray(state['q'], float)

    def joint_states(self, msg):
        values = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if all(n in values and np.isfinite(values[n]) for n in JOINTS):
            self.update(q=[values[n] for n in JOINTS], sample_time=time.monotonic(),
                        velocity=[float(velocities[n]) if n in velocities and np.isfinite(velocities[n]) else None for n in JOINTS],
                        ros_stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def diagnostics(self, msg):
        rows = {name: dict(zip(values.interface_names, values.values)) for name, values in zip(msg.joint_names, msg.interface_values)}
        self.update(rows=rows, diagnostic_time=time.monotonic())


class NativeStack:
    def __init__(self, model, folder, namespace, mock, with_rviz):
        self.model, self.directory, self.namespace = model, Path(folder), namespace
        self.mock, self.with_rviz = mock, with_rviz
        self.children, self.control_children, self.streams = [], [], []
        self.context = self.node = None
        self.pump = None
        self.close_requested = False
        self.control_generation = 0
        self.remaps = ['--ros-args', '-r', '__ns:=' + namespace]
        for topic in ('tf', 'tf_static', 'joint_states', 'dynamic_joint_states', 'display_planned_path'):
            self.remaps += ['-r', '/' + topic + ':=' + namespace + '/' + topic]

    def spawn(self, name, package, executable, args=(), control=False, params=None):
        from ament_index_python.packages import get_package_prefix
        command = [str(Path(get_package_prefix(package)) / 'lib' / package / executable), *args, *self.remaps]
        if params:
            command += ['--params-file', str(params)]
        stream = (self.directory / (name + '.log')).open('a')
        self.streams.append(stream)
        env = dict(os.environ, A6Y_SESSION_PID=str(os.getpid()), ROS_LOG_DIR=str(self.directory / 'ros_logs'))
        command = ['/usr/bin/python3', str(Path(__file__).with_name('process_guard.py')), str(os.getpid()), *command]
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                                 cwd=self.directory, env=env)
        self.children.append((name, child))
        if control:
            self.control_children.append((name, child))
        return child

    def start(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('a6y_session', namespace=self.namespace, context=self.context)
        params, self.moveit_params = write_moveit_config(self.model, self.directory)
        self.spawn('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher', params=params)
        self.spawn('world_tf', 'tf2_ros', 'static_transform_publisher', ['--frame-id', 'world', '--child-frame-id', 'base_link'])
        self.start_control(enabled=False)
        self.spawn('move_group', 'moveit_ros_move_group', 'move_group', params=params)
        if self.with_rviz:
            config = rviz_config(self.namespace)
            config['Panels'].insert(0, {'Class': 'a6y_rviz_panel/PlanningPanel', 'Name': 'A6Y MoveIt / ros2_control'})
            config['Panels'] = [p for p in config['Panels'] if p['Class'] != 'rviz_common/Views']
            config['Window Geometry'].update(Width=1440, Height=1000)
            manager = config['Visualization Manager']
            manager['Views']['Current'].update(Distance=1.65, **{'Focal Point': {'X': .18, 'Y': 0., 'Z': .3}})
            manager['Displays'] = [d for d in manager['Displays'] if d['Name'] != 'Feedback status and joint axes']
            manager['Displays'] += [
                {'Class': 'rviz_default_plugins/InteractiveMarkers', 'Name': 'MoveIt TCP target', 'Enabled': True,
                 'Value': True, 'Show Axes': False, 'Show Descriptions': False,
                 'Interactive Markers Namespace': self.namespace + '/tcp_target'},
                {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'MoveIt IK goal', 'Enabled': True, 'Value': True,
                 'Topic': {'Value': self.namespace + '/model_preview', 'Depth': 1}},
                {'Class': 'moveit_rviz_plugin/Trajectory', 'Name': 'MoveIt planned trajectory', 'Enabled': True, 'Value': True,
                 'Robot Description': 'robot_description', 'Trajectory Topic': self.namespace + '/display_planned_path',
                 'Loop Animation': True, 'Robot Alpha': .5, 'State Display Time': '0.05 s'}]
            path = self.directory / 'native.rviz'
            path.write_text(yaml.safe_dump(config, sort_keys=False))
            self.spawn('rviz2', 'rviz2', 'rviz2', ['-d', str(path)], params=params)

    def start_control(self, enabled, initial=None):
        # A fresh service name prevents clients for a destroyed manager from
        # sending queued requests to its replacement during DDS discovery.
        self.manager_name = 'controller_manager_' + str(self.control_generation)
        self.control_generation += 1
        path = write_controller_config(self.model, self.directory, self.namespace, self.mock, enabled, initial,
                                       manager_name=self.manager_name)
        self.spawn('controller_manager', 'controller_manager', 'ros2_control_node',
                   ['--ros-args', '-r', 'controller_manager:__node:=' + self.manager_name], control=True, params=path)
        for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
            args = [name, '--controller-manager', self.namespace + '/' + self.manager_name, '--controller-manager-timeout', '25']
            if not enabled and name != 'joint_state_broadcaster':
                args.append('--inactive')
            self.spawn('spawner_' + name, 'controller_manager', 'spawner', args, control=True)

    @staticmethod
    def terminate(children, pump=None):
        # One deadline per phase for the whole group, not 6 seconds per child.
        # The guard forwards signals and reaps the native executable.
        for sig, seconds in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
            pending = [child for _, child in children if child.poll() is None]
            for child in pending:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + seconds
            while any(child.poll() is None for child in pending) and time.monotonic() < deadline:
                if pump:
                    pump()
                else:
                    time.sleep(.02)
        return [(name, child.poll()) for name, child in children]

    def stop_control(self):
        old = self.control_children
        self.terminate(old, self.pump)
        if any(child.poll() is None for _, child in old):
            raise RuntimeError('旧控制进程未退出，已拒绝启动第二个控制器')
        self.children = [c for c in self.children if c not in old]
        self.control_children = []

    def check(self):
        for name, child in self.children:
            # Finite action clients have their own result/exit supervision.
            if name in getattr(self, 'separately_supervised_children', ()):
                continue
            code = child.poll()
            if code is None or (name.startswith('spawner_') and code == 0):
                continue
            if name == 'rviz2' and code == 0:
                if not getattr(self, 'close_reported', False):
                    self.close_requested = self.close_reported = True
                continue
            raise RuntimeError(f'{name} 退出（{code}），见 {self.directory / (name + ".log")}')

    def close(self):
        self.terminate(self.children)
        for stream in self.streams:
            stream.close()
        if self.node:
            self.node.destroy_node()
        if self.context and self.context.ok():
            self.context.shutdown()
