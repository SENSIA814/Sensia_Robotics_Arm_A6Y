#!/usr/bin/env python3
"""Local role selector and live readout. Opening the UI never opens a serial port."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from model import JOINTS, load_model, load_roles, ROLE_ORDER
from scanner import inventory
from viewer_session import ViewSession

from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QColor, QDesktopServices
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QButtonGroup, QComboBox,
                            QGridLayout, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
                            QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

STYLE = '''
QWidget { background: #0d141c; color: #e0e9f2; font-size: 13px; }
QLabel#title { font-size: 27px; font-weight: 600; }
QLabel#eyebrow { color: #67dfba; font-size: 12px; }
QLabel#note { color: #9aadc0; padding: 5px 0; }
QLabel#status { color: #74e4c3; background: #132c28; padding: 13px; border-radius: 6px; }
QLabel#error { color: #ffbb7b; }
QPushButton { background: #192634; border: 1px solid #34485a; border-radius: 6px; padding: 10px 16px; }
QPushButton:hover { border-color: #6f9cba; }
QPushButton:checked { background: #183b32; border: 2px solid #67dfba; }
QPushButton:disabled { color: #667a8e; border-color: #253444; }
QPushButton#start { background: #67dfba; color: #09251d; font-weight: 600; }
QPushButton#start:disabled { background: #244b42; color: #7b958d; }
QPushButton#stop { color: #ff9a9a; }
QComboBox { background: #192634; padding: 9px; border: 1px solid #34485a; }
QTableWidget { background: #111d28; alternate-background-color: #162331; gridline-color: #273a4c; border: 1px solid #2b3d50; }
QHeaderView::section { background: #203040; border: none; padding: 10px 5px; color: #a9c0d5; }
QTableWidget::item { padding: 5px; }
QToolTip { background: #253a4b; color: #e0e9f2; border: 1px solid #67859c; }
'''


class ViewWindow(QMainWindow):
    def __init__(self, session=None):
        super().__init__()
        self.session = session or ViewSession()
        self.setWindowTitle('A6Y · 只读反馈与 RViz 零位检查')
        self.resize(1180, 850)
        self.setStyleSheet(STYLE)
        self.model = None
        self._closing = False
        self._was_running = False
        self._inventory_time = 0.
        main = QWidget()
        layout = QVBoxLayout(main)
        layout.setContentsMargins(26, 22, 26, 22)
        layout.setSpacing(12)
        for text, name in [('A6Y  /  MODEL CHECK', 'eyebrow'), ('只读反馈与模型核对', 'title')]:
            label = QLabel(text)
            label.setObjectName(name)
            layout.addWidget(label)
        note = QLabel('选择一条机械臂，核对电机零位、关节方向与安装位置。此入口仅发送状态查询。')
        note.setObjectName('note')
        layout.addWidget(note)
        grid = QGridLayout()
        self.group = QButtonGroup(self)
        self.buttons = {}
        for i, role in enumerate(ROLE_ORDER):
            arm = self.session.roles[role]
            button = QPushButton(arm['name'] + '\nUSB ' + arm['usb_serial'])
            button.setMinimumHeight(80)
            button.setCheckable(True)
            button.clicked.connect(lambda _=False, r=role: self.select_role(r))
            self.group.addButton(button)
            self.buttons[role] = button
            grid.addWidget(button, i // 2, i % 2)
        layout.addLayout(grid)
        actions = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItem('实时只读反馈 + RViz', False)
        self.mode.addItem('仅预览 URDF 零姿态（无硬件）', True)
        self.mode.currentIndexChanged.connect(self.update_mode)
        actions.addWidget(self.mode)
        self.start_button = QPushButton('选择机械臂后启动')
        self.start_button.setObjectName('start')
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start)
        actions.addWidget(self.start_button)
        self.stop_button = QPushButton('停止显示并释放串口')
        self.stop_button.setObjectName('stop')
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.session.stop)
        actions.addWidget(self.stop_button)
        self.folder_button = QPushButton('打开本次记录')
        self.folder_button.setEnabled(False)
        self.folder_button.clicked.connect(self.open_report)
        actions.addWidget(self.folder_button)
        layout.addLayout(actions)
        self.status = QLabel('请选择运动端／示教端及左臂／右臂，启动前不会连接电机。')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.error = QLabel('')
        self.error.setWordWrap(True)
        self.error.setObjectName('error')
        layout.addWidget(self.error)
        self.table = QTableWidget(7, 7)
        self.table.setHorizontalHeaderLabels(['模型关节 / ID', '电机位置 °', '电机位置 rad',
                                              '模型位置 ° / mm', '显示换算 s × k × q + b', 'URDF 轴', '反馈状态'])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMinimumHeight(320)
        for i in range(7):
            self.table.setRowHeight(i, 39)
            for c in range(7):
                self.table.setItem(i, c, QTableWidgetItem('—'))
        layout.addWidget(self.table, 1)
        footer = QLabel('黄色箭头：URDF 关节正轴；RGB 坐标轴：X / Y / Z。夹爪模型坐标是单指位移，单位 mm。\n'
                        '四条臂暂共用所提供的 A5 URDF，均以各自 base_link 为基准；左右臂不自动镜像。\n'
                        '方向、比例沿用原程序并标为待核对；不自动抵消零偏、不限幅。发现不一致，停止后返回上一步标定。')
        footer.setWordWrap(True)
        footer.setObjectName('note')
        layout.addWidget(footer)
        self.setCentralWidget(main)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.refresh_inventory()

    def selected_role(self):
        return next((role for role, button in self.buttons.items() if button.isChecked()), None)

    def select_role(self, role):
        try:
            self.model = load_model(role, self.session.roles)
            self.error.clear()
            self.start_button.setEnabled(True)
            self.update_mode()
            self.render_rows({}, False, False)
        except Exception as exc:
            self.model = None
            self.error.setText(str(exc))
            self.start_button.setEnabled(False)

    def update_mode(self):
        role = self.selected_role()
        if role:
            self.start_button.setText(('预览 · ' if self.mode.currentData() else '启动 · ') + self.session.roles[role]['name'])
        if self.mode.currentData():
            self.status.setText('零姿态预览：仅显示 URDF 中 q = 0 的模型；这不是电机反馈，也不代表标定通过。')
        else:
            self.status.setText('实时模式：使用选定 USB 的新鲜反馈；缺失反馈时暂停姿态更新。')

    def start(self):
        try:
            self.session.start(self.selected_role(), preview=bool(self.mode.currentData()))
            self._was_running = True
            self.error.clear()
            self.set_running(True)
        except Exception as exc:
            self.error.setText(str(exc))

    def set_running(self, running):
        for button in self.buttons.values():
            button.setEnabled(not running)
        self.mode.setEnabled(not running)
        self.start_button.setEnabled(not running and self.model is not None)
        self.stop_button.setEnabled(running)

    def refresh_inventory(self):
        for arm in inventory(self.session.roles):
            self.buttons[arm['role']].setText(arm['name'] + '\nUSB ' + arm['usb_serial'] + '\n' + arm['connection_message'])
        self._inventory_time = time.monotonic()

    def render_rows(self, rows, preview, running):
        if self.model is None:
            return
        now = time.monotonic()
        for i, name in enumerate(JOINTS):
            entry = self.model['joints'][name]
            row = rows.get(name, {})
            feedback = row.get('feedback')
            q = row.get('q')
            stale = bool(feedback) and (not running or now - row['sample_monotonic'] > .4)
            if preview:
                q = 0.
            unit = ' mm' if entry['type'] == 'prismatic' else '°'
            value = (q * 1000 if entry['type'] == 'prismatic' else math.degrees(q)) if q is not None else None
            state = '等待新反馈'
            if preview:
                state = '模型预览 · 无实测'
            elif feedback:
                code = feedback['state']
                state = ('已失能' if code == 0 else '已使能' if code == 1 else f'异常 0x{code:X}')
                if row['out_of_limits']:
                    state += ' / 超限'
                if stale:
                    state = '过期 / ' + state
            elif running:
                state = '等待／缺失反馈'
            texts = [f'{"夹爪" if i == 6 else "J" + str(i + 1)} / {entry["motor_id"]}',
                     f'{feedback["position_deg"]:.2f}' if feedback else '—',
                     f'{feedback["position_rad"]:.5f}' if feedback else '—',
                     f'{value:.2f}{unit}' if value is not None else '—',
                     f'{entry["sign"]:+g} × {entry["position_scale"]:g} × q {entry["offset"]:+g}',
                     ' '.join(f'{x:g}' for x in entry['axis']), state]
            tip = f'{name}\nchild: {entry["child"]}\norigin: {entry["origin"]}\nURDF limits: [{entry["lower"]}, {entry["upper"]}]\n显示映射待实机核对'
            if feedback:
                tip += f'\n采样: {feedback["received_at"]}\n原始计数: {feedback["position_raw"]}\nPMAX: {feedback["position_pmax_rad"]} rad'
            for col, text in enumerate(texts):
                item = self.table.item(i, col)
                item.setText(text)
                item.setToolTip(tip)
                warn = stale or row.get('out_of_limits') or (feedback and feedback['state'] != 0)
                item.setForeground(QColor('#ffbd82' if warn else '#e0e9f2'))

    def tick(self):
        state = self.session.snapshot()
        running = state['running']
        if running or self._was_running:
            self.status.setText(state['status'])
            self.error.setText(state['error'] or '')
            self.render_rows(state['rows'], state['preview'], running)
            self.folder_button.setEnabled(bool(state.get('report')))
        if self._was_running and not running:
            # Joinable thread can briefly still be writing its final state.
            if self.session.worker and self.session.worker.is_alive():
                return
            self._was_running = False
            self.set_running(False)
            self.group.setExclusive(False)
            for button in self.buttons.values():
                button.setChecked(False)
            self.group.setExclusive(True)
            self.start_button.setEnabled(False)
            self.start_button.setText('重新选择机械臂后启动')
        if time.monotonic() - self._inventory_time > 2 and not running:
            self.refresh_inventory()
        if self._closing and not running:
            self.close()

    def open_report(self):
        path = self.session.snapshot().get('report')
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).parent)))

    def closeEvent(self, event):
        if self.session.worker and self.session.worker.is_alive():
            self._closing = True
            self.session.stop()
            self.status.setText('正在停止查询、关闭 RViz 并释放串口…')
            event.ignore()
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description='A6Y 只读反馈与 RViz 检查窗口')
    parser.add_argument('--arm', choices=ROLE_ORDER)
    parser.add_argument('--cli', action='store_true', help='命令行运行；省略 --arm 时显示选择菜单')
    parser.add_argument('--preview', action='store_true', help='只显示 URDF 零姿态，不连接硬件')
    parser.add_argument('--dry-run', action='store_true', help='仅核对配置；不启动 ROS、RViz 或串口')
    parser.add_argument('--no-rviz', action='store_true', help='CLI 仅发布状态和 TF，不打开 RViz')
    parser.add_argument('--duration', type=float, help='CLI 运行指定秒数后停止')
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration 必须为有限正数')
    if (args.no_rviz or args.duration is not None) and not args.cli:
        parser.error('--no-rviz / --duration 只能与 --cli 一起使用')
    roles = load_roles()
    if args.dry_run:
        selected = [args.arm] if args.arm else ROLE_ORDER
        for role in selected:
            model = load_model(role, roles)
            print(json.dumps({k: model[k] for k in ('role', 'arm', 'urdf_path', 'urdf_sha256', 'joints', 'mapping_verified')}, ensure_ascii=False, indent=2))
        print('DRY RUN：没有连接串口，没有启动 ROS 或 RViz。')
        return 0
    if args.cli:
        role = args.arm
        if role is None:
            if not sys.stdin.isatty():
                parser.error('非交互终端必须指定 --arm')
            print('\nA6Y · 选择一条机械臂（仅查询状态）')
            for i, name in enumerate(ROLE_ORDER, 1):
                print(f'  {i}. {roles[name]["name"]}  USB {roles[name]["usb_serial"]}')
            value = input('输入 1～4，或 q 退出：').strip()
            if value.lower() == 'q':
                return 0
            if value not in ('1', '2', '3', '4'):
                parser.error('无效选择')
            role = ROLE_ORDER[int(value) - 1]
        if not args.no_rviz and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            parser.error('当前无图形桌面，请在本机桌面终端运行；仅发布可加 --no-rviz')
        session = ViewSession(roles)
        signal.signal(signal.SIGINT, lambda *_: session.stop())
        signal.signal(signal.SIGTERM, lambda *_: session.stop())
        session.start(role, args.preview, not args.no_rviz)
        start, last = time.monotonic(), ''
        while session.worker.is_alive():
            state = session.snapshot()
            if state['status'] != last:
                print(state['status'], flush=True)
                last = state['status']
            if args.duration and time.monotonic() - start >= args.duration:
                session.stop()
            session.worker.join(.2)
        state = session.snapshot()
        print(state['error'] or state['status'])
        print('记录：', state['report'])
        return 1 if state['error'] else 0
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('当前没有图形桌面，请在本机桌面终端运行同一启动脚本。')
    app = QApplication(sys.argv[:1])
    window = ViewWindow(ViewSession(roles))
    if args.arm:
        window.buttons[args.arm].click()
    if args.preview:
        window.mode.setCurrentIndex(1)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
