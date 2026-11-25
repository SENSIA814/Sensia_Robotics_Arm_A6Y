#!/usr/bin/env python3
"""Role-selected local MoveIt terminal; planning and execution are explicit."""
import argparse
import math
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np
import pinocchio as pin
from PyQt5.QtCore import QTimer, QUrl, Qt
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
                            QGridLayout, QHBoxLayout, QLabel, QMainWindow, QPushButton,
                            QVBoxLayout, QWidget, QScrollArea, QSplitter, QFileDialog)

from motion_model import Kinematics, ROLE_ORDER, load_roles
from native_model import native_model as planning_model
from native_session import PlanningSession
from scanner import inventory
from arm_view import STYLE
from monitor_view import MonitorView


class MoveItWindow(QMainWindow):
    def __init__(self, session=None):
        super().__init__()
        self.roles = load_roles()
        self.session = session or PlanningSession(self.roles)
        self.setWindowTitle('A6Y · 原版 MoveIt / ros2_control / RViz')
        self.setStyleSheet(STYLE)
        self.resize(1640, 1000)
        self.setMinimumSize(1150, 740)
        self._closing = self._was_running = False
        self._inventory_time = 0.
        self.kin = None
        main = QWidget()
        box = QVBoxLayout(main)
        box.setContentsMargins(16, 16, 16, 16)
        box.setSpacing(9)
        heading = QLabel('末端位姿规划与执行')
        heading.setObjectName('title')
        box.addWidget(heading)
        hint = QLabel('RViz 拖动末端 → MoveIt 规划并预览 → ros2_control 执行 · 用户原程序移植')
        hint.setWordWrap(True)
        hint.setObjectName('eyebrow')
        box.addWidget(hint)
        grid = QGridLayout()
        self.group = QButtonGroup(self)
        self.buttons = {}
        for i, role in enumerate(ROLE_ORDER):
            button = QPushButton(self.roles[role]['name'])
            button.setMinimumHeight(65)
            button.setCheckable(True)
            self.group.addButton(button)
            self.buttons[role] = button
            button.clicked.connect(lambda _=False, r=role: self.select_role(r))
            grid.addWidget(button, i // 2, i % 2)
        box.addLayout(grid)
        row = QGridLayout()
        self.mode = QComboBox()
        self.mode.addItem('模拟规划执行（不连接硬件）', True)
        self.mode.addItem('真机反馈与规划执行', False)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        row.addWidget(self.mode, 0, 0)
        self.start_button = QPushButton('选择机械臂后启动')
        self.start_button.setObjectName('start')
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start)
        row.addWidget(self.start_button, 0, 1)
        self.close_session = QPushButton('关闭会话（已接管则先回零）')
        self.close_session.setEnabled(False)
        self.close_session.clicked.connect(self.session.stop)
        row.addWidget(self.close_session, 1, 0)
        self.logs = QPushButton('查看本次日志')
        self.logs.setEnabled(False)
        self.logs.clicked.connect(self.open_logs)
        row.addWidget(self.logs, 1, 1)
        self.log_directory = QPushButton('打开日志目录 / CSV')
        self.log_directory.clicked.connect(self.open_log_directory)
        row.addWidget(self.log_directory, 2, 0)
        self.history_logs = QPushButton('查看历史日志…')
        self.history_logs.clicked.connect(self.open_history_logs)
        row.addWidget(self.history_logs, 2, 1)
        box.addLayout(row)
        self.home_on_start = QCheckBox('启动后自动规划回零并保持（真机会使能、运动）')
        self.home_on_start.setChecked(True)
        self.home_on_start.setToolTip('四种机械臂均适用：J1～J6 回到零位；回零成功后将末端曲线设为零参考。取消勾选可只读启动。')
        self.home_on_start.toggled.connect(lambda _: self.update_start_label())
        box.addWidget(self.home_on_start)
        from native_model import collision_profiles
        collision_config = collision_profiles()
        collision_row = QHBoxLayout()
        collision_row.addWidget(QLabel('碰撞检查'))
        self.collision_profile = QComboBox()
        for key, profile in collision_config['profiles'].items():
            self.collision_profile.addItem(profile['label'], key)
        self.collision_profile.setCurrentIndex(self.collision_profile.findData(collision_config['default_profile']))
        collision_row.addWidget(self.collision_profile)
        self.collision_hint = QLabel()
        self.collision_hint.setWordWrap(True)
        collision_row.addWidget(self.collision_hint, 1)
        def collision_hint():
            from native_model import resolved_collision_policy
            self.collision_hint.setText(resolved_collision_policy(self.selected(), self.collision_profile.currentData())['description'])
        self.refresh_collision_hint = collision_hint
        self.collision_profile.currentIndexChanged.connect(collision_hint)
        collision_hint()
        box.addLayout(collision_row)
        self.permission = QCheckBox('已核对零位与方向，允许真机执行')
        self.permission.setToolTip('自动回零启动会授予执行许可；取消许可会停止当前动作。只读启动时需手动勾选。')
        self.permission.setEnabled(False)
        self.permission.toggled.connect(lambda value: self.session.send('permit', value))
        box.addWidget(self.permission)
        self.execution_hint = QLabel()
        self.execution_hint.setWordWrap(True)
        box.addWidget(self.execution_hint)
        self.status = QLabel('先选臂；默认模拟模式。启动将自动规划回零并保持，到位后等待下一步指令。')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)
        box.addWidget(self.status)
        self.hardware_status = QLabel('未连接')
        self.hardware_status.setWordWrap(True)
        box.addWidget(self.hardware_status)
        self.error = QLabel('')
        self.error.setObjectName('error')
        self.error.setWordWrap(True)
        box.addWidget(self.error)
        self.current = QLabel('当前 TCP：—\n关节角：—')
        self.current.setWordWrap(True)
        self.current.setMinimumHeight(65)
        box.addWidget(self.current)
        self.target = QLabel('目标 TCP：拖动 RViz 中的绿色末端目标和六轴操作柄')
        self.target.setWordWrap(True)
        box.addWidget(self.target)
        box.addStretch(1)
        planning = QGridLayout()
        self.reset_button = QPushButton('目标复位到当前末端')
        self.reset_button.clicked.connect(lambda: self.session.send('reset'))
        planning.addWidget(self.reset_button, 0, 0, 1, 2)
        planning.addWidget(QLabel('速度'), 1, 0)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setDecimals(0)
        self.speed.setSuffix(' %')
        self.speed.setValue(10)
        planning.addWidget(self.speed, 1, 1)
        planning.addWidget(QLabel('加速度'), 1, 2)
        self.acceleration = QDoubleSpinBox()
        self.acceleration.setRange(1, 100)
        self.acceleration.setDecimals(0)
        self.acceleration.setSuffix(' %')
        self.acceleration.setValue(10)
        planning.addWidget(self.acceleration, 1, 3)
        self.position_only = QCheckBox('仅位置（可选；勾选后关闭 RPY 姿态控制）')
        self.position_only.setChecked(False)
        box.addWidget(self.position_only)
        for control in (self.speed, self.acceleration):
            control.valueChanged.connect(lambda _: self.session.send('settings', self.settings()))
        self.position_only.toggled.connect(lambda _: self.session.send('settings', self.settings()))
        self.plan_button = QPushButton('① 规划并预览')
        self.plan_button.clicked.connect(lambda: self.session.send('plan', self.settings()))
        planning.addWidget(self.plan_button, 2, 0, 1, 2)
        self.execute_button = QPushButton('② 执行预览轨迹')
        self.execute_button.setObjectName('start')
        self.execute_button.clicked.connect(lambda: self.session.send('execute'))
        planning.addWidget(self.execute_button, 2, 2, 1, 2)
        box.addLayout(planning)
        gripper = QGridLayout()
        gripper.addWidget(QLabel('夹爪目标开度'), 0, 0)
        self.gripper_percent = QDoubleSpinBox()
        self.gripper_percent.setRange(0, 100)
        self.gripper_percent.setDecimals(0)
        self.gripper_percent.setSuffix(' %')
        self.gripper_percent.valueChanged.connect(lambda value: self.session.send('gripper_target', value))
        gripper.addWidget(self.gripper_percent, 0, 1)
        self.gripper_close = QPushButton('闭合目标')
        self.gripper_close.clicked.connect(lambda: self.session.send('gripper_target', 0))
        self.gripper_open = QPushButton('张开目标')
        self.gripper_open.clicked.connect(lambda: self.session.send('gripper_target', 100))
        self.gripper_plan = QPushButton('① 夹爪规划并预览')
        self.gripper_plan.clicked.connect(lambda: self.session.send('gripper_plan', self.settings()))
        gripper.addWidget(self.gripper_close, 1, 0)
        gripper.addWidget(self.gripper_open, 1, 1)
        gripper.addWidget(self.gripper_plan, 1, 2)
        box.addLayout(gripper)
        stopping = QHBoxLayout()
        self.stop_button = QPushButton('停止轨迹 · 保持当前位置')
        self.stop_button.setObjectName('stop')
        self.stop_button.clicked.connect(lambda: self.session.send('stop'))
        stopping.addWidget(self.stop_button)
        self.release_button = QPushButton('规划并执行回零 · J1～J6 = 0')
        self.release_button.clicked.connect(lambda: self.session.send('home'))
        stopping.addWidget(self.release_button)
        self.emergency_button = QPushButton('已支撑机械臂 · 紧急失能（跳过回零）')
        self.emergency_button.clicked.connect(lambda: self.session.send('emergency_release'))
        note = QLabel('自动回零启动：J1～J6 到零位后保持；末端曲线以本次回零后的位姿为零参考。\n'
                      '使用原程序 MoveIt 关节限值，速度／加速度默认各 10%；修改后重新规划。执行由 100 Hz ros2_control 完成。\n'
                      '回零按钮执行 J1～J6 = 0 后保持；退出时未接管则直接关闭，已接管则先回零再失能。红／绿／蓝圆环控制 Roll／Pitch／Yaw。\n'
                      '回零失败或被停止会取消退出；紧急失能和故障不执行回零。失能前需支撑机械臂。\n'
                      '碰撞检查使用提供的 URDF 网格与规划场景；未建模的桌面、人员和周边物体不会自动被识别。')
        note.setObjectName('note')
        note.setWordWrap(True)
        box.addWidget(note)
        scrolling_controls = QScrollArea()
        scrolling_controls.setWidgetResizable(True)
        scrolling_controls.setWidget(main)
        control_pane = QWidget()
        control_layout = QVBoxLayout(control_pane)
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(scrolling_controls, 1)
        # Stop and release stay visible even when the controls need scrolling.
        control_layout.addLayout(stopping)
        control_layout.addWidget(self.emergency_button)
        self.monitor = MonitorView()
        split = QSplitter(Qt.Horizontal)
        split.addWidget(control_pane)
        split.addWidget(self.monitor)
        split.setSizes([650, 990])
        split.setChildrenCollapsible(False)
        self.setCentralWidget(split)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.refresh_inventory()
        self.tick()

    def selected(self):
        return next((role for role, button in self.buttons.items() if button.isChecked()), None)

    def settings(self):
        return {'velocity_scaling': self.speed.value()/100, 'acceleration_scaling': self.acceleration.value()/100,
                'position_only': self.position_only.isChecked()}

    def select_role(self, role):
        try:
            self.refresh_collision_hint()
            self.kin = Kinematics(planning_model(role, self.roles))
            self.start_button.setEnabled(True)
            self.update_start_label()
            self.error.clear()
        except Exception as exc:
            self.error.setText(str(exc))
            self.start_button.setEnabled(False)

    def mode_changed(self):
        self.permission.setChecked(False)

    def update_start_label(self):
        role = self.selected()
        self.start_button.setText(('启动并回零 · ' if self.home_on_start.isChecked() else '只读启动 · ') +
                                 self.roles[role]['name'] if role else '选择机械臂后启动')

    def start(self):
        try:
            self.permission.setChecked(False)
            self.session.start(self.selected(), mock=bool(self.mode.currentData()),
                               collision_profile=self.collision_profile.currentData(),
                               home_on_start=self.home_on_start.isChecked())
            self._was_running = True
            self.tick()
        except Exception as exc:
            self.error.setText(str(exc))

    def refresh_inventory(self):
        for arm in inventory(self.roles):
            brief = {'unavailable': 'USB 已识别 · 串口不可访问', 'identity_mismatch': '串口身份不符 · 查看提示',
                     'duplicate_serial': 'USB 序列号重复 · 查看提示', 'alias_conflict': '串口别名冲突 · 查看提示',
                     'dfu': 'USB 下载模式 · 无可用串口'}.get(arm.get('connection_state'), arm['connection_message'])
            self.buttons[arm['role']].setText(f'{arm["name"]}\nUSB {arm["usb_serial"]}\n{brief}')
            self.buttons[arm['role']].setToolTip(arm['connection_message'])
        self._inventory_time = time.monotonic()

    @staticmethod
    def pose_text(matrix):
        matrix = np.asarray(matrix)
        xyz = matrix[:3, 3] * 1000
        rpy = np.rad2deg(pin.rpy.matrixToRpy(matrix[:3, :3]))
        return f'XYZ = {xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f} mm    RPY = {rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}°'

    def tick(self):
        s = self.session.snapshot()
        running, ready, busy = s['running'], s['ready'], s['busy']
        h = s.get('hardware', {})
        executing = h.get('executing', False)
        busy = busy or s.get('home_active', False)
        for button in self.buttons.values():
            button.setEnabled(not running)
        self.mode.setEnabled(not running)
        self.home_on_start.setEnabled(not running)
        self.collision_profile.setEnabled(not running)
        self.start_button.setEnabled(not running and self.selected() is not None)
        self.close_session.setEnabled(running and not s.get('closing_requested', False))
        self.plan_button.setEnabled(ready and not busy and not executing)
        self.speed.setEnabled(ready and not busy and not executing)
        self.acceleration.setEnabled(ready and not busy and not executing)
        self.position_only.setEnabled(ready and not busy and not executing)
        for key, widget in [('velocity_scaling', self.speed), ('acceleration_scaling', self.acceleration), ('position_only', self.position_only)]:
            if key in s and not widget.hasFocus():
                widget.blockSignals(True)
                if key == 'position_only':
                    widget.setChecked(s[key])
                else:
                    widget.setValue(100*s[key])
                widget.blockSignals(False)
        self.reset_button.setEnabled(ready and not busy and not executing)
        self.permission.setEnabled(ready and not s['mock'])
        self.permission.blockSignals(True)
        self.permission.setChecked(s['permitted'])
        self.permission.blockSignals(False)
        for control in (self.gripper_percent, self.gripper_close, self.gripper_open, self.gripper_plan):
            control.setEnabled(ready and not busy and not executing)
        if s.get('gripper_percent') is not None and not self.gripper_percent.hasFocus():
            self.gripper_percent.blockSignals(True)
            self.gripper_percent.setValue(s['gripper_percent'])
            self.gripper_percent.blockSignals(False)
        self.execute_button.setText('② 执行夹爪预览轨迹' if s.get('plan_group') == 'gripper' else '② 执行机械臂预览轨迹')
        self.execute_button.setEnabled(ready and not busy and not executing and s['plan_ready'] and (s['mock'] or s['permitted']))
        if not ready:
            execution_hint = '执行不可用：请先启动机械臂会话并等待就绪。'
        elif busy or executing:
            execution_hint = '执行不可用：当前操作尚未结束。'
        elif not s['mock'] and not s['permitted']:
            execution_hint = ('规划已通过；执行按钮等待授权：请勾选上方“允许真机执行”。' if s['plan_ready'] else
                              '真机执行尚未授权：勾选上方许可后，规划、预览，再点击执行。')
        elif not s['plan_ready']:
            execution_hint = '执行不可用：请重新规划。修改目标或参数、轨迹超过 30 秒后需重新规划。'
        else:
            group = '夹爪' if s.get('plan_group') == 'gripper' else '机械臂'
            execution_hint = f'可执行{group}：检查预览后点击执行；控制器将在点击后自动激活。'
        self.execution_hint.setText(execution_hint)
        self.execution_hint.setStyleSheet('color:#f1bf64;' if ready and not s['mock'] and not s['permitted'] else '')
        self.execute_button.setToolTip(execution_hint)
        self.stop_button.setEnabled(running)
        self.release_button.setEnabled(ready and not s.get('home_active', False) and (s['mock'] or s['permitted']))
        self.emergency_button.setEnabled(running and (h.get('enabled', False) or executing or s.get('home_active', False)))
        self.logs.setEnabled(bool(s.get('directory')))
        self.log_directory.setEnabled(bool(s.get('directory')))
        self.monitor.refresh(self.session, s)
        if running or self._was_running:
            self.status.setText(s['status'] + '\n' + s.get('preview_status', '') + '\n' + s.get('wait_note', ''))
            self.error.setText((s['error'] or '') + ('\n' + s['warning'] if s.get('warning') else ''))
            self.hardware_status.setText(('模拟：' if s['mock'] else '真机：') + h.get('status', '正在启动'))
            q = h.get('q')
            if q is not None and self.kin and time.monotonic() - h['sample_time'] <= .25:
                self.current.setText('绝对 TCP（日志保留）：' + self.pose_text(self.kin.fk(q)) + '\n关节角：' +
                                     '  '.join(f'J{i+1} {math.degrees(v):.2f}°' for i, v in enumerate(q[:6])) +
                                     f'    夹爪 {q[6]*1000:.2f} mm')
            else:
                self.current.setText('当前 TCP / 关节角：等待新鲜反馈')
            if s.get('target') is not None:
                self.target.setText('目标 TCP：' + self.pose_text(s['target']))
        if self._was_running and not running and not self.session.worker.is_alive():
            self._was_running = False
            self.permission.setChecked(False)
            self.group.setExclusive(False)
            for b in self.buttons.values():
                b.setChecked(False)
            self.group.setExclusive(True)
            self.start_button.setEnabled(False)
        if not running and time.monotonic() - self._inventory_time > 2:
            self.refresh_inventory()
        if s.get('close_failed'):
            self._closing = False
        if self._closing and not running:
            self.close()

    def open_logs(self):
        self.monitor.logs.set_folder(self.session.snapshot().get('directory'))
        self.monitor.tabs.setCurrentWidget(self.monitor.logs)

    def open_log_directory(self):
        path = self.session.snapshot().get('directory')
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def open_history_logs(self):
        path = QFileDialog.getExistingDirectory(self, '选择以机械臂名称开头的会话日志目录',
                                               str(self.session.report_dir) if hasattr(self.session, 'report_dir') else '')
        if path:
            self.monitor.logs.set_folder(path)
            self.monitor.tabs.setCurrentWidget(self.monitor.logs)

    def closeEvent(self, event):
        if self.session.worker and self.session.worker.is_alive():
            self._closing = True
            self.session.stop()
            event.ignore()
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description='四臂 MoveIt / Pinocchio / RViz 选择窗口')
    parser.add_argument('--arm', choices=ROLE_ORDER)
    parser.add_argument('--mode', choices=('mock', 'real'), default='mock')
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--cli', action='store_true', help='终端选择并启动模拟；规划／执行使用 RViz 面板')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--read-only-start', action='store_true', help='取消默认启动回零，仅启动只读反馈')
    parser.add_argument('--collision-profile', choices=('bare', 'full'), help='按所选机械臂应用裸机碰撞规则 / 恢复完整检查')
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration 必须为有限正数')
    if args.check_config:
        from ament_index_python.packages import get_package_prefix
        for package in ('moveit_ros_move_group', 'moveit_planners_ompl', 'moveit_ros_visualization', 'controller_manager', 'joint_trajectory_controller', 'a6_y_ros2_control', 'rmw_cyclonedds_cpp', 'rviz2'):
            print(package, get_package_prefix(package))
        for role in [args.arm] if args.arm else ROLE_ORDER:
            model = planning_model(role, collision_profile=args.collision_profile)
            Kinematics(model)
            print(role, model['arm']['usb_serial'], model['urdf_sha256'], model['collision_policy']['label'])
        print(f'Pinocchio {pin.__version__}；配置检查通过。没有连接电机或启动 ROS 节点。')
        return 0
    if args.cli:
        if args.mode == 'real':
            parser.error('真机执行使用默认图形选择窗口，以便明确允许执行并随时停止')
        role = args.arm
        if role is None:
            if not sys.stdin.isatty():
                parser.error('非交互终端必须指定 --arm')
            roles = load_roles()
            for i, r in enumerate(ROLE_ORDER, 1):
                print(i, roles[r]['name'])
            choice = input('选择 1～4，q 退出：').strip()
            if choice == 'q':
                return 0
            if choice not in ('1', '2', '3', '4'):
                parser.error('无效选择')
            role = ROLE_ORDER[int(choice)-1]
        if not args.no_rviz and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            parser.error('当前无图形桌面，测试可加 --no-rviz')
        session = PlanningSession()
        signal.signal(signal.SIGINT, lambda *_: session.stop())
        signal.signal(signal.SIGTERM, lambda *_: session.stop())
        session.start(role, mock=True, with_rviz=not args.no_rviz, collision_profile=args.collision_profile,
                      home_on_start=not args.read_only_start)
        start, last = time.monotonic(), None
        while session.worker.is_alive():
            s = session.snapshot()
            text = s['error'] or s['status']
            if text != last:
                print(text, flush=True)
                last = text
            if args.duration and time.monotonic() - start >= args.duration:
                session.stop()
            session.worker.join(.2)
        print(session.snapshot()['error'] or '已退出')
        return 1 if session.snapshot()['error'] else 0
    if args.no_rviz or args.duration:
        parser.error('--no-rviz / --duration 仅用于 --cli')
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('请在 Ubuntu 图形桌面的终端运行此脚本')
    app = QApplication(sys.argv[:1])
    window = MoveItWindow()
    window.home_on_start.setChecked(not args.read_only_start)
    if args.collision_profile:
        window.collision_profile.setCurrentIndex(window.collision_profile.findData(args.collision_profile))
    if args.arm:
        window.buttons[args.arm].click()
    window.mode.setCurrentIndex(0 if args.mode == 'mock' else 1)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
