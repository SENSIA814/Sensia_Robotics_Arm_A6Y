"""ROS visualization processes only; no control managers or command interfaces."""
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

from model import JOINTS, ROOT


def rviz_config(namespace):
    config = yaml.safe_load((ROOT / 'src/a6_y_controllers/config/a5_readonly.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Background Color'] = '20; 26; 35'
    for display in manager['Displays']:
        if display['Class'] == 'rviz_default_plugins/RobotModel':
            display['Description Topic']['Value'] = namespace + '/robot_description'
        if display['Class'] == 'rviz_default_plugins/TF':
            display['Show Names'] = False
            display['Marker Scale'] = .12
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Feedback status and joint axes',
        'Enabled': True, 'Value': True,
        'Topic': {'Value': namespace + '/markers', 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Depth': 1}})
    manager['Views']['Current'].update(Distance=1.25, Pitch=.5, Yaw=-.9)
    return config


class RosDisplay:
    def __init__(self, model, directory, namespace, with_rviz=True):
        self.model, self.directory, self.namespace = model, Path(directory), namespace
        self.with_rviz = with_rviz
        self.children = []
        self.streams = []
        self.context = self.node = None

    def __enter__(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import JointState
            from visualization_msgs.msg import MarkerArray
            from ament_index_python.packages import get_package_prefix
            self.context = Context()
            rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('readonly_feedback', namespace=self.namespace, context=self.context)
            self.joints_pub = self.node.create_publisher(JointState, self.namespace + '/joint_states', 1)
            self.markers_pub = self.node.create_publisher(MarkerArray, self.namespace + '/markers', 1)
            params = self.directory / 'robot_description.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                'robot_description': self.model['description'], 'publish_frequency': 30.0}}}), encoding='utf-8')
            config = self.directory / 'readonly.rviz'
            config.write_text(yaml.safe_dump(rviz_config(self.namespace), sort_keys=False), encoding='utf-8')
            self.remaps = ['--ros-args', '-r', '__ns:=' + self.namespace,
                           '-r', '/tf:=' + self.namespace + '/tf',
                           '-r', '/tf_static:=' + self.namespace + '/tf_static']
            rsp = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
            self._spawn('robot_state_publisher', [str(rsp), *self.remaps, '--params-file', str(params)])
            if self.with_rviz:
                rviz = Path(get_package_prefix('rviz2')) / 'lib/rviz2/rviz2'
                self._spawn('rviz2', [str(rviz), '-d', str(config), *self.remaps])
            deadline = time.monotonic() + 6.
            while self.joints_pub.get_subscription_count() == 0:
                if not self.alive():
                    raise RuntimeError('RViz 在启动期间已关闭')
                if time.monotonic() > deadline:
                    raise RuntimeError('ROS 模型节点未连接到关节反馈，请检查本机 DDS 通信及日志')
                time.sleep(.05)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def _spawn(self, name, command):
        stream = (self.directory / (name + '.log')).open('w', encoding='utf-8')
        self.streams.append(stream)
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((name, child))

    def alive(self):
        for name, child in self.children:
            code = child.poll()
            if code is not None:
                if name == 'rviz2' and code == 0:
                    return False
                raise RuntimeError(f'{name} 已退出（{code}），详见 {self.directory / (name + ".log")}')
        return True

    def publish(self, positions, status, warning=False):
        from sensor_msgs.msg import JointState
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        stamp = self.node.get_clock().now().to_msg()
        if positions is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name, msg.position = list(JOINTS), list(positions)
            # Velocity and effort are intentionally absent, not fabricated zeros.
            self.joints_pub.publish(msg)
        markers = MarkerArray()
        text = Marker()
        text.header.frame_id, text.header.stamp = 'base_link', stamp
        text.ns, text.id, text.type = 'feedback_status', 0, Marker.TEXT_VIEW_FACING
        text.pose.position.z, text.pose.orientation.w = .65, 1.0
        text.scale.z = .027
        text.color.r, text.color.g, text.color.b, text.color.a = (1., .3, .25, 1.) if warning else (.4, 1., .8, 1.)
        text.text = self.model['role'] + '\n' + status
        markers.markers.append(text)
        for index, name in enumerate(JOINTS):
            entry = self.model['joints'][name]
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = entry['child'], stamp
            marker.ns, marker.id, marker.type = 'positive_joint_axes', index, Marker.ARROW
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = .004, .009, .012
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .75, .2, 1.
            marker.frame_locked = True
            norm = sum(x*x for x in entry['axis']) ** .5
            marker.points = [Point(), Point(**dict(zip(('x', 'y', 'z'), [x / norm * .09 for x in entry['axis']])))]
            markers.markers.append(marker)
        self.markers_pub.publish(markers)

    def __exit__(self, *_):
        # Signal only our own sessions; never kill other ROS or calibration jobs.
        for _, child in reversed(self.children):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for _, child in reversed(self.children):
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    child.wait(timeout=2)
                    break
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(child.pid, sig)
                    except ProcessLookupError:
                        pass
            child.wait(timeout=2)
        for stream in self.streams:
            stream.close()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.shutdown()
