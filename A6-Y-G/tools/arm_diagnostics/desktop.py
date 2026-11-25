#!/usr/bin/env python3
"""Native Qt desktop scanner. No HTTP server, browser, or web view is used."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys

try:
    from PyQt5.QtCore import Qt, QTimer, QUrl
    from PyQt5.QtGui import QColor, QDesktopServices, QFont
    from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox,
                                QFileDialog, QFrame, QGridLayout, QHBoxLayout,
                                QHeaderView, QLabel, QMainWindow, QMessageBox,
                                QProgressBar, QPushButton, QScrollArea,
                                QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
except ImportError:
    print("缺少本地窗口组件 PyQt5，请安装 python3-pyqt5 后重试。", file=sys.stderr)
    raise SystemExit(2)

import yaml
from scanner import DEFAULT_REPORTS, DEFAULT_ROLES, inventory, load_roles
from session import ScanSession
from position_feedback import position_text


STYLE = """
QWidget { background: #0c1117; color: #e2eaf3; font-size: 13px; }
QMainWindow, QScrollArea, QScrollArea > QWidget > QWidget { background: #0c1117; }
QScrollArea { border: none; }
QFrame#sidebar { background: #111820; border-right: 1px solid #283341; }
QFrame#sidebar QLabel { background: transparent; }
QLabel#brand { font-size: 27px; font-weight: 700; }
QLabel#nav { color: #61dfb9; background: #18322b; border: 1px solid #304b41; border-radius: 6px; padding: 13px; }
QLabel#eyebrow { color: #61dfb9; font-size: 11px; }
QLabel#heading { font-size: 27px; font-weight: 600; }
QLabel#muted { color: #8c9bad; font-size: 12px; }
QLabel#section { font-size: 15px; font-weight: 600; }
QLabel#notice { color: #bfd8d0; background: #142321; border: 1px solid #2b453d; border-radius: 6px; padding: 12px; }
QLabel#error { color: #efbc76; font-size: 12px; }
QFrame#armCard { background: #151d27; border: 1px solid #2b3745; border-radius: 8px; }
QFrame#armCard[chosen="true"] { background: #152a24; border: 1px solid #61dfb9; }
QFrame#armCard QLabel, QFrame#armCard QCheckBox { background: transparent; border: none; }
QCheckBox { font-size: 16px; font-weight: 600; spacing: 10px; }
QCheckBox::indicator { width: 17px; height: 17px; border: 1px solid #647386; border-radius: 3px; background: #17202a; }
QCheckBox::indicator:checked { background: #61dfb9; border: 4px solid #61dfb9; image: none; }
QCheckBox:disabled { color: #8997a6; }
QPushButton { background: #1a2531; border: 1px solid #344254; border-radius: 6px; padding: 9px 15px; }
QPushButton:hover { border-color: #8399af; }
QPushButton:focus { border: 1px solid #61dfb9; }
QPushButton:disabled { background: #17212b; border-color: #263340; color: #627284; }
QPushButton#primary { background: #61dfb9; border-color: #61dfb9; color: #09291f; font-weight: 600; }
QPushButton#primary:hover { background: #85edcb; }
QPushButton#primary:disabled { background: #245346; color: #87a69b; border-color: #245346; }
QPushButton#stop { color: #ff8f9b; }
QPushButton#zero { background: #e8ad61; color: #30200b; border-color: #e8ad61; font-weight: 600; }
QPushButton#zero:disabled { background: #4c3b25; color: #98846a; border-color: #4c3b25; }
QComboBox { background: #151e28; border: 1px solid #344254; border-radius: 5px; padding: 9px 15px; min-width: 96px; }
QComboBox QAbstractItemView { background: #17222f; selection-background-color: #284a40; }
QProgressBar { border: none; background: #24333e; max-height: 4px; min-height: 4px; }
QProgressBar::chunk { background: #61dfb9; }
QTableWidget { background: #121b25; alternate-background-color: #16202b; border: 1px solid #2a3847; border-radius: 5px; gridline-color: #263543; selection-background-color: #23463d; }
QHeaderView::section { background: #1b2734; color: #9aabba; border: none; border-bottom: 1px solid #344252; padding: 11px 8px; font-size: 11px; }
QTableWidget::item { padding: 7px; }
QTableWidget::item:selected { background: #23463d; }
QTableCornerButton::section { background: #1b2734; border: none; }
QScrollBar:vertical { background: #101922; width: 10px; }
QScrollBar::handle:vertical { background: #384b5c; min-height: 30px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: #101922; height: 10px; }
QScrollBar::handle:horizontal { background: #384b5c; min-width: 30px; border-radius: 4px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QToolTip { background: #253443; color: #dce8f5; border: 1px solid #4c647a; }
"""

RESULTS = {
    "pending": ("待扫描", "#8f9daf"), "not_scanned": ("未扫描", "#8f9daf"),
    "ok": ("每轮响应", "#61dfb9"), "missing": ("无响应", "#ff8f9b"),
    "intermittent": ("间歇响应", "#efbc76"), "state_warning": ("状态需检查", "#efbc76"),
    "zero_pending": ("待标定", "#8f9daf"), "zero_checking": ("正在检查", "#8f9daf"),
    "zero_ready": ("检查通过", "#8f9daf"), "zero_ok": ("零位已验证", "#61dfb9"),
    "zero_unverified": ("写入未确认", "#efbc76"), "zero_failed": ("检查失败", "#ff8f9b"),
    "zero_not_written": ("未写入", "#8f9daf"),
}


def label(text, name=None, wrap=False):
    item = QLabel(text)
    item.setTextFormat(Qt.PlainText)
    if name:
        item.setObjectName(name)
    item.setWordWrap(wrap)
    return item


class ScanWindow(QMainWindow):
    def __init__(self, session):
        super().__init__()
        self.session = session
        self._running = False
        self._closing = False
        self._last_state = None
        self._report = None
        self.cards = {}
        self.checkboxes = {}
        self.connections = {}
        self.setWindowTitle("A6Y · 七电机扫描与零位标定")
        self.setMinimumSize(960, 640)
        self.resize(1340, 920)
        self.setStyleSheet(STYLE)
        self._build()
        self.refresh_devices()
        self.poll()
        self.timer = QTimer(self)
        self.timer.setInterval(150)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        self.device_timer = QTimer(self)
        self.device_timer.setInterval(2000)
        self.device_timer.timeout.connect(self.refresh_devices_if_idle)
        self.device_timer.start()

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(190)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(20, 28, 20, 24)
        side.setSpacing(14)
        side.addWidget(label("A6Y", "brand"))
        side.addWidget(label("ROBOT WORKSPACE", "muted"))
        side.addSpacing(24)
        side.addWidget(label("扫描 / 零位标定", "nav"))
        side.addSpacing(20)
        side.addWidget(label("本次功能", "section"))
        for text in ("只读扫描       1–4 台臂", "零位标定       1 台臂", "每臂电机       7 台", "身份绑定       USB 序列号"):
            side.addWidget(label(text, "muted"))
        side.addSpacing(20)
        side.addWidget(label("通信参数", "section"))
        for text in ("CAN             1 Mbps", "串口              115200", "适配器           CANable2"):
            side.addWidget(label(text, "muted"))
        side.addStretch()
        side.addWidget(label("● 本地桌面程序", "eyebrow"))
        side.addWidget(label("USB 串口 · 本地执行", "muted"))
        outer.addWidget(sidebar)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)
        body = QVBoxLayout(content)
        body.setContentsMargins(28, 25, 28, 25)
        body.setSpacing(14)
        body.addWidget(label("机械臂工作台   /   通信诊断", "muted"))
        body.addSpacing(10)
        body.addWidget(label("A6Y WORKSPACE     /     MOTOR TOOLS", "eyebrow"))
        body.addWidget(label("七电机扫描与零位标定", "heading"))
        body.addWidget(label("检查电机反馈，或将手动摆好的姿态写为电机零位。两项功能互斥运行。", "muted", True))
        mode_row = QHBoxLayout()
        mode_row.addWidget(label("本次功能", "section"))
        self.mode = QComboBox()
        self.mode.addItem("只读扫描", "scan")
        self.mode.addItem("写入电机零位", "zero")
        mode_row.addWidget(self.mode)
        mode_row.addStretch()
        body.addLayout(mode_row)
        body.addSpacing(8)
        section = QHBoxLayout()
        section.addWidget(label("01   选择本次机械臂", "section"))
        section.addStretch()
        self.adapter_count = label("", "muted")
        section.addWidget(self.adapter_count)
        self.refresh_button = QPushButton("刷新连接")
        self.refresh_button.clicked.connect(self.refresh_devices)
        section.addWidget(self.refresh_button)
        body.addLayout(section)
        grid = QGridLayout()
        grid.setSpacing(12)
        self.arm_grid = grid
        self._card_columns = 4
        for index, (role, arm) in enumerate(self.session.roles.items()):
            card = QFrame()
            card.setObjectName("armCard")
            card.setProperty("chosen", False)
            stack = QVBoxLayout(card)
            stack.setContentsMargins(15, 15, 15, 15)
            stack.setSpacing(9)
            stack.addWidget(label("MOTION · 运动端" if role.startswith("motion") else "TEACHING · 示教端", "muted"))
            checkbox = QCheckBox(arm["name"])
            checkbox.setAccessibleName(arm["name"])
            checkbox.toggled.connect(lambda checked, r=role: self.arm_toggled(r, checked))
            stack.addWidget(checkbox)
            stack.addWidget(label(f"USB {arm['usb_serial']}", "muted"))
            connection = label("", "muted", True)
            connection.setMinimumWidth(150)
            stack.addWidget(connection)
            grid.addWidget(card, 0, index)
            self.cards[role] = card
            self.checkboxes[role] = checkbox
            self.connections[role] = connection
        body.addLayout(grid)
        controls = QHBoxLayout()
        self.selection_count = label("", "muted")
        controls.addWidget(self.selection_count, 1)
        self.select_all = QPushButton("选择全部")
        self.select_all.clicked.connect(self.toggle_all)
        controls.addWidget(self.select_all)
        self.passes = QComboBox()
        for count, text in ((3, "3 轮 · 快速"), (10, "10 轮 · 标准"), (20, "20 轮 · 复检")):
            self.passes.addItem(text, count)
        self.passes.setCurrentIndex(1)
        controls.addWidget(self.passes)
        self.stop_button = QPushButton("停止扫描")
        self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(self.stop_scan)
        self.stop_button.hide()
        controls.addWidget(self.stop_button)
        self.start_button = QPushButton("开始只读扫描  →")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_scan)
        controls.addWidget(self.start_button)
        self.zero_button = QPushButton("写入所选臂零位")
        self.zero_button.setObjectName("zero")
        self.zero_button.clicked.connect(self.start_zero)
        self.zero_button.hide()
        controls.addWidget(self.zero_button)
        body.addLayout(controls)
        self.notice = label("仅发送状态查询，不启用电机。扫描前请停止占用所选机械臂串口的其他程序。", "notice", True)
        body.addWidget(self.notice)
        self.error_label = label("", "error", True)
        self.error_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.error_label.hide()
        body.addWidget(self.error_label)
        results_header = QHBoxLayout()
        results_header.addWidget(label("02   执行结果", "section"))
        results_header.addStretch()
        self.online_count = label("— 电机响应", "eyebrow")
        results_header.addWidget(self.online_count)
        body.addLayout(results_header)
        self.status_label = label("请选择本次要扫描的机械臂", "muted", True)
        body.addWidget(self.status_label)
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.hide()
        body.addWidget(self.progress)
        self.empty_label = label("01 ── 02 ── 03 ── 04 ── 05 ── 06 ── 07\n\n等待选择机械臂\n每台机械臂检查 ID 1–7；打开窗口不会扫描电机。", "muted", True)
        self.empty_label.setAlignment(Qt.AlignCenter)
        self.empty_label.setMinimumHeight(190)
        body.addWidget(self.empty_label)
        body.addWidget(label("位置为电机轴角度，按此前读取的量程换算；结束后保留最后采样，不持续更新。夹爪显示角度，非开口宽度。", "muted", True))
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels(["机械臂 / 电机", "CAN ID", "响应 / 轮数", "检查结果", "位置 °", "位置 rad", "状态码", "MOS / 转子 °C", "最新反馈报文"])
        self.table.verticalHeader().hide()
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMinimumHeight(290)
        self.table.hide()
        body.addWidget(self.table, 1)
        report_actions = QHBoxLayout()
        self.report_path = label("完成后自动保存 JSON 报告。每次任务结束后需要重新选择机械臂。", "muted", True)
        self.report_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        report_actions.addWidget(self.report_path, 1)
        self.save_button = QPushButton("报告另存为…")
        self.save_button.clicked.connect(self.export_report)
        self.save_button.setEnabled(False)
        report_actions.addWidget(self.save_button)
        self.folder_button = QPushButton("打开报告文件夹")
        self.folder_button.clicked.connect(self.open_report_folder)
        self.folder_button.setEnabled(False)
        report_actions.addWidget(self.folder_button)
        body.addLayout(report_actions)
        body.addWidget(label("全部七台电机每轮响应、状态为 0 且无异常报文，才标记本轮通信检查通过。\n扫描只确认本次通信；不能排除重复 ID、间歇接触或其他硬件问题。", "muted", True))
        self.mode.currentIndexChanged.connect(self.mode_changed)

    def mode_changed(self):
        for box in self.checkboxes.values():
            box.setChecked(False)
        is_zero = self.mode.currentData() == "zero"
        self.select_all.setVisible(not is_zero)
        self.passes.setVisible(not is_zero)
        self.start_button.setVisible(not is_zero)
        self.zero_button.setVisible(is_zero)
        self.notice.setText(
            "先将所选臂的 J1～J6 和夹爪摆到零位，并保持支撑、失能、静止，再点击写入。程序会写入电机并回读核验，不驱动机械臂运动。停止只中止后续电机，已写入的零位不会撤回。"
            if is_zero else "仅发送状态查询，不启用电机。扫描前请停止占用所选机械臂串口的其他程序。")
        self.update_selection()

    def arm_toggled(self, role, checked):
        if checked and self.mode.currentData() == "zero":
            for other, box in self.checkboxes.items():
                if other != role:
                    box.blockSignals(True)
                    box.setChecked(False)
                    box.blockSignals(False)
        self.update_selection()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not hasattr(self, "arm_grid"):
            return
        columns = 2 if self.width() < 1150 else 4
        if columns != self._card_columns:
            for card in self.cards.values():
                self.arm_grid.removeWidget(card)
            for index, card in enumerate(self.cards.values()):
                self.arm_grid.addWidget(card, index // columns, index % columns)
            self._card_columns = columns

    def selected_roles(self):
        return [role for role, box in self.checkboxes.items() if box.isChecked()]

    def update_selection(self, *_):
        selected = self.selected_roles()
        self.selection_count.setText(f"已选择 {len(selected)} 台 · {len(selected) * 7} 台电机" if selected else "已选择 0 台 · 请先选择机械臂")
        self.start_button.setEnabled(bool(selected) and not self._running and not self._closing)
        self.zero_button.setEnabled(len(selected) == 1 and not self._running and not self._closing)
        self.zero_button.setText(f"已摆好零位，写入{self.session.roles[selected[0]]['name']}" if len(selected) == 1 else "请先选择一条臂")
        if not self._running and not (self._last_state or {}).get("selected"):
            if self.mode.currentData() == "zero":
                self.status_label.setText(f"请摆好{self.session.roles[selected[0]]['name']}的零位，再点击写入" if selected else "请选择一条需要标定的机械臂")
                self.empty_label.setText("01 ── 02 ── 03 ── 04 ── 05 ── 06 ── 07\n\nJ1～J6 与夹爪均包含在本次标定中\n点击写入后开始；仅选择机械臂不会发送指令。")
            else:
                self.status_label.setText("选择完成，点击开始只读扫描" if selected else "请选择本次要扫描的机械臂")
                self.empty_label.setText("01 ── 02 ── 03 ── 04 ── 05 ── 06 ── 07\n\n每台机械臂检查 ID 1～7\n点击扫描后开始；打开窗口不会扫描电机。")
        self.select_all.setText("清空选择" if len(selected) == 4 else "选择全部")
        for role, card in self.cards.items():
            card.setProperty("chosen", role in selected)
            card.style().unpolish(card)
            card.style().polish(card)

    def toggle_all(self):
        choose = len(self.selected_roles()) < len(self.checkboxes)
        for box in self.checkboxes.values():
            box.setChecked(choose)

    def refresh_devices(self):
        devices = inventory(self.session.roles)
        for arm in devices:
            self.connections[arm["role"]].setText(('● ' if arm['present'] else '○ ') + arm['connection_message'])
            self.connections[arm["role"]].setToolTip(arm["port"] + '\n' + arm['connection_message'])
            color = '#61dfb9' if arm['present'] else '#efbc76' if arm['usb_detected'] else '#8c9bad'
            self.connections[arm["role"]].setStyleSheet(f'color: {color};')
        self.adapter_count.setText(f"USB 识别 {sum(a['usb_detected'] for a in devices)} / {len(devices)} · 串口可用 {sum(a['present'] for a in devices)}")
        self.update_selection()

    def refresh_devices_if_idle(self):
        if not self._running and not self._closing:
            self.refresh_devices()

    def start_scan(self):
        if self.mode.currentData() != "scan" or self._running or self._closing or not self.selected_roles():
            return
        try:
            self.session.start(self.selected_roles(), self.passes.currentData())
            # Set this before polling: a disconnected-device run can finish
            # before the first timer tick and must still clear the selection.
            self._running = True
            self.error_label.hide()
            self.poll()
        except (ValueError, RuntimeError) as exc:
            self.error_label.setText(str(exc))
            self.error_label.show()

    def start_zero(self):
        selected = self.selected_roles()
        if self.mode.currentData() != "zero" or self._running or self._closing or len(selected) != 1:
            return
        try:
            self.session.start_zero(selected[0])
            self._running = True
            self.error_label.hide()
            self.poll()
        except (ValueError, RuntimeError) as exc:
            self.error_label.setText(str(exc))
            self.error_label.show()

    def stop_scan(self):
        self.session.cancel.set()
        self.stop_button.setEnabled(False)
        self.status_label.setText("正在停止后续操作并保存报告；已发送的写零位不会撤回…" if self.mode.currentData() == "zero" else "正在停止扫描并保存部分报告…")

    def poll(self):
        state = self.session.snapshot()
        if state != self._last_state:
            self._render(state)
            self._last_state = state
        if self._closing and not state["running"] and (not self.session.worker or not self.session.worker.is_alive()):
            self.close()

    def _render(self, state):
        was_running = self._running
        self._running = state["running"]
        if was_running and not self._running:
            for box in self.checkboxes.values():
                box.setChecked(False)
        for box in self.checkboxes.values():
            box.setEnabled(not self._running and not self._closing)
        for item in (self.select_all, self.refresh_button, self.passes, self.mode):
            item.setEnabled(not self._running and not self._closing)
        self.update_selection()
        self.start_button.setText("正在扫描…" if self._running else "开始只读扫描  →")
        is_zero = state.get("operation") == "zero"
        self.stop_button.setText("停止后续标定" if is_zero else "停止扫描")
        if is_zero and self._running:
            self.zero_button.setText("正在写入与核验…")
        self.stop_button.setVisible(self._running)
        self.stop_button.setEnabled(self._running and not self.session.cancel.is_set())
        self.progress.setVisible(self._running)
        self.progress.setRange(0, max(1, state["total"]))
        self.progress.setValue(state["progress"])
        self.status_label.setText(state["message"] + (f" · {state['progress']} / {state['total']} {'台已验证' if is_zero else '次查询'}" if self._running else ""))
        report = state["report"]
        self._report = report
        rows = []
        for role in state["selected"]:
            arm = next(a for a in report["arms"] if a["role"] == role) if report else self.session.roles[role]
            for motor_id in range(1, 8):
                motor = arm["motors"][motor_id - 1] if report else state["motors"].get(f"{role}:{motor_id}")
                rows.append((arm, motor_id, motor))
        self.empty_label.setVisible(not rows)
        self.table.setVisible(bool(rows))
        self.table.setRowCount(len(rows))
        self.table.horizontalHeaderItem(2).setText("反馈 / 查询" if is_zero else "响应 / 轮数")
        self.table.horizontalHeaderItem(3).setText("标定结果" if is_zero else "检查结果")
        self.online_count.setText(f"{sum(bool(m and m['hits']) for _, _, m in rows)} / {len(rows)} 电机响应" if rows else "— 电机响应")
        if is_zero and rows:
            self.online_count.setText(f"{sum(bool(m and m.get('verified')) for _, _, m in rows)} / {len(rows)} 零位已验证")
        for row, (arm, motor_id, motor) in enumerate(rows):
            last = motor.get("last") if motor else None
            text, color = RESULTS[motor["result"] if motor else "pending"]
            if self._running and not is_zero and motor and motor["attempts"]:
                text, color = f"已查询 {motor['attempts']} 次", "#8f9daf"
            values = [f"{arm['name']} · {'J' + str(motor_id) if motor_id < 7 else '夹爪'}",
                      f"0x{motor_id:02X} → 0x{motor_id + 16:02X}",
                      f"{motor['hits'] if motor else 0} / {(motor['attempts'] if motor else 0) if is_zero else state.get('passes', '—')}", text,
                      f"{last['position_deg']:.2f}" if last and last.get("position_deg") is not None else ("未配置量程" if last else "—"),
                      f"{last['position_rad']:.4f}" if last and last.get("position_rad") is not None else "—",
                      ", ".join(f"0x{s:X}" for s in motor["states"]) if motor and motor["states"] else "—",
                      f"{last['mos_temperature_c']} / {last['rotor_temperature_c']}" if last else "—",
                      last["raw_hex"] if last else "—"]
            for col, value in enumerate(values):
                cell = self.table.item(row, col)
                if cell is None:
                    cell = QTableWidgetItem()
                    self.table.setItem(row, col, cell)
                cell.setText(value)
                cell.setForeground(QColor(color if col == 3 else "#c3d0de"))
                tooltip = value
                if last and col in (4, 5):
                    tooltip = f"最近采样：{last['received_at']}\n电机位置：{position_text(last)}"
                    scale = motor.get("position_scale")
                    if scale:
                        tooltip += f"\nPMAX：±{scale['pmax_rad']:.9g} rad\n量程读取时间：{scale['measured_at']}\n量程来源：{scale['source']}"
                    if motor.get("before_samples"):
                        tooltip += f"\n标定前：{position_text(motor['before_samples'][-1])}"
                    if motor.get("after_samples"):
                        tooltip += f"\n写入后：{position_text(motor['after_samples'][-1])}"
                elif last and col == 8:
                    tooltip = last["received_at"]
                cell.setToolTip(tooltip)
        errors = [state["error"]] if state["error"] else []
        if report:
            if report.get("position_scale_error"):
                errors.append(report["position_scale_error"])
            for arm in report["arms"]:
                if arm["error"]:
                    errors.append(f"{arm['name']}：{arm['error']}")
                if arm["unexpected_frames"]:
                    errors.append(f"{arm['name']}：{arm['unexpected_frames']} 个未匹配反馈，请核查 ID 与其他总线通信。")
                if any(any(s != 0 for s in m["states"]) for m in arm["motors"]):
                    errors.append(f"{arm['name']}：出现非零状态码，请核查电机状态。")
        self.error_label.setText("\n".join(errors))
        self.error_label.setVisible(bool(errors))
        self.save_button.setEnabled(bool(report) and not self._running)
        self.folder_button.setEnabled(bool(state["report_path"]) and not self._running)
        if report:
            self.report_path.setText(f"{report['finished_at'].replace('T', ' ')}\n" +
                                     (f"报告已保存：{state['report_path']}" if state["report_path"] else "报告保存失败，可点击“报告另存为”。"))
        elif self._running:
            self.report_path.setText("正在采集本轮新反馈；完成或停止后自动保存 JSON。")

    def export_report(self):
        if not self._report:
            return
        initial = self._last_state.get("report_path") or str(Path(self.session.report_dir) / "arm_scan_report.json")
        filename, _ = QFileDialog.getSaveFileName(self, "另存扫描报告", initial, "JSON 报告 (*.json)")
        if filename:
            try:
                Path(filename).write_text(json.dumps(self._report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            except OSError as exc:
                QMessageBox.warning(self, "保存失败", str(exc))

    def open_report_folder(self):
        path = self._last_state.get("report_path") if self._last_state else None
        if path and not QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).parent))):
            QMessageBox.information(self, "报告目录", str(Path(path).parent))

    def closeEvent(self, event):
        worker = self.session.worker
        if worker and worker.is_alive():
            self._closing = True
            self.session.cancel.set()
            self.start_button.setEnabled(False)
            self.zero_button.setEnabled(False)
            self.mode.setEnabled(False)
            self.status_label.setText("正在停止后续操作并保存报告，完成后关闭窗口…")
            event.ignore()
        else:
            self.timer.stop()
            self.device_timer.stop()
            event.accept()


def main():
    parser = argparse.ArgumentParser(description="A6Y 本地桌面扫描窗口（无需浏览器）")
    parser.add_argument("--roles", type=Path, default=DEFAULT_ROLES)
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORTS)
    args = parser.parse_args()
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.environ.get("QT_QPA_PLATFORM") == "offscreen"):
        parser.exit(2, "当前终端没有图形桌面连接。请在本机桌面的终端运行此命令；纯终端扫描可用 --cli --arms 指定机械臂。\n")
    try:
        session = ScanSession(load_roles(args.roles), args.report_dir)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.exit(2, f"角色配置读取失败：{exc}\n")
    app = QApplication([sys.argv[0]])
    app.setApplicationName("A6Y 机械臂诊断")
    app.setFont(QFont("Noto Sans CJK SC", 10))
    window = ScanWindow(session)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
