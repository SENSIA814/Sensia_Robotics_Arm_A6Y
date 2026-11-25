"""Qt-only scrolling charts and incremental log tails; never commands hardware."""
from collections import deque
from datetime import datetime
import math
from pathlib import Path
import re
import time
from zoneinfo import ZoneInfo

from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                            QPlainTextEdit, QPushButton, QTabWidget, QVBoxLayout, QWidget)

COLORS = ('#66d9ef', '#ffbc70', '#9be28c', '#caadff', '#ff86aa', '#f0df75', '#b8c8d9')
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


class StripChart(QWidget):
    def __init__(self, title, labels, unit, wrap=False, include_range=None):
        super().__init__()
        self.title, self.labels, self.unit, self.wrap = title, labels, unit, wrap
        self.include_range = include_range
        self.samples = deque(maxlen=2400)
        self.window_s = 30
        self.setMinimumSize(310, 215)

    def append(self, elapsed, values):
        self.samples.append((elapsed, values))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#111d2c'))
        painter.setPen(QColor('#e5edf5'))
        painter.setFont(QFont('Sans', 10, QFont.Bold))
        painter.drawText(12, 21, self.title + '  [' + self.unit + ']')
        painter.setFont(QFont('Sans', 9))
        columns = 3 if len(self.labels) > 3 else len(self.labels)
        rows = math.ceil(len(self.labels)/columns)
        last = self.samples[-1][1] if self.samples else [None]*len(self.labels)
        for i, label in enumerate(self.labels):
            painter.setPen(QColor(COLORS[i]))
            value = '—' if last[i] is None else f'{last[i]:.2f}'
            painter.drawText(12+(i % columns)*(self.width()-18)//columns, 42+(i//columns)*17, f'{label}  {value}')
        rect = QRectF(62, 51+(rows-1)*17, self.width()-78, self.height()-80-(rows-1)*17)
        end = self.samples[-1][0] if self.samples else 0.
        start = end-self.window_s
        visible = [(t, v) for t, v in self.samples if t >= start]
        values = [v for _, point in visible for v in point if v is not None and math.isfinite(v)]
        low, high = (min(values), max(values)) if values else (-1., 1.)
        if self.include_range:
            low, high = min(low, self.include_range[0]), max(high, self.include_range[1])
        padding = 0. if self.include_range else max((high-low)*.1, .05 if self.unit == 'mm/s' else .5)
        low, high = low-padding, high+padding
        painter.setFont(QFont('Sans', 8))
        divisions = 4 if self.include_range else 3
        for i in range(divisions+1):
            y = rect.top()+rect.height()*i/divisions
            painter.setPen(QColor('#29384b'))
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.setPen(QColor('#a8b8cb'))
            painter.drawText(QRectF(0, y-8, 56, 16), Qt.AlignRight | Qt.AlignVCenter, f'{high-(high-low)*i/divisions:.2f}')
        for fraction in (0., .5, 1.):
            x = rect.left()+rect.width()*fraction
            painter.drawText(QRectF(x-28, rect.bottom()+5, 56, 18), Qt.AlignCenter, f'{-self.window_s*(1-fraction):.0f} s')
        painter.save()
        painter.setClipRect(rect)
        # Limit drawing work by pixel buckets, preserving min/max and missing-data gaps.
        stride = max(1, len(visible)//max(1, int(rect.width())))
        for axis in range(len(self.labels)):
            path = QPainterPath()
            previous = None
            points = []
            for offset in range(0, len(visible), stride):
                bucket = visible[offset:offset+stride]
                if stride == 1 or any(point[1][axis] is None for point in bucket):
                    points.extend(bucket)
                else:
                    selected = {0, len(bucket)-1,
                                min(range(len(bucket)), key=lambda j: bucket[j][1][axis]),
                                max(range(len(bucket)), key=lambda j: bucket[j][1][axis])}
                    points.extend(bucket[j] for j in sorted(selected))
            for t, value in points:
                v = value[axis]
                if v is None or not math.isfinite(v):
                    previous = None
                    continue
                x = rect.left()+(t-start)/self.window_s*rect.width()
                y = rect.bottom()-(v-low)/(high-low)*rect.height()
                gap = previous is None or t-previous[0] > .25 or (self.wrap and abs(v-previous[1]) > 180)
                if gap:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
                previous = (t, v)
            painter.setPen(QPen(QColor(COLORS[axis]), 1.4))
            painter.drawPath(path)
        painter.restore()
        if not values:
            painter.setPen(QColor('#c0cfde'))
            painter.drawText(rect, Qt.AlignCenter, '等待有效反馈 · 缺失值不画曲线')


class LogViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.folder, self.positions = None, {}
        box = QVBoxLayout(self)
        bar = QHBoxLayout()
        self.source = QComboBox()
        self.source.addItem('全部：应用结果 + ROS 原始输出', '*')
        self.source.addItem('应用结果（规划 / 碰撞 / 执行）', 'application.log')
        self.search = QLineEdit()
        self.search.setPlaceholderText('筛选文字，例如：碰撞 / ERROR / planning')
        self.follow = QCheckBox('自动滚动')
        self.follow.setChecked(True)
        bar.addWidget(self.source, 2); bar.addWidget(self.search, 2); bar.addWidget(self.follow)
        box.addLayout(bar)
        self.path = QLabel('启动后自动保存日志；也可打开以前会话的日志目录。')
        self.path.setWordWrap(True)
        self.path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        box.addWidget(self.path)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(2000)
        self.text.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.text.setFont(QFont('Monospace', 9))
        box.addWidget(self.text)
        note = QLabel('显示最近 2000 行；筛选时从每个文件末尾重新读取。完整原始输出保存在目录内，未删改。\n'
                      '多源按读取顺序汇总；行首为来源，ROS 原始行保留其时间戳。采样数据在 telemetry.csv。')
        note.setWordWrap(True)
        box.addWidget(note)
        self.source.currentIndexChanged.connect(self.reset_tail)
        self.search.textChanged.connect(self.reset_tail)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)

    def set_folder(self, folder):
        folder = Path(folder) if folder else None
        if folder == self.folder:
            return
        self.folder = folder
        self.source.blockSignals(True)
        while self.source.count() > 2:
            self.source.removeItem(2)
        self.source.blockSignals(False)
        self.path.setText(str(folder) if folder else '会话尚未启动')
        self.reset_tail()

    def reset_tail(self):
        self.positions.clear()
        self.text.clear()
        self.poll()

    def poll(self):
        if not self.folder or not self.folder.exists():
            return
        sources = sorted(self.folder.glob('*.log'))
        known = {self.source.itemData(i) for i in range(self.source.count())}
        for source in sources:
            if source.name not in known:
                self.source.addItem('夹爪开合记录' if source.name == 'gripper.log' else 'ROS · '+source.stem, source.name)
        chosen, keyword = self.source.currentData(), self.search.text().casefold()
        lines = []
        for source in sources:
            if chosen != '*' and source.name != chosen:
                continue
            if chosen == '*' and source.name == 'gripper.log':
                continue  # These entries are already present in application.log.
            try:
                size = source.stat().st_size
                position, remainder = self.positions.get(source, (max(0, size-65536), b''))
                if size < position:
                    position, remainder = 0, b''
                with source.open('rb') as stream:
                    stream.seek(position)
                    chunk = stream.read(32768)
                    position = stream.tell()
                parts = (remainder+chunk).split(b'\n')
                self.positions[source] = (position, parts.pop()[-65536:])
                for raw in parts:
                    line = ANSI.sub('', raw.decode('utf-8', errors='replace'))
                    if not keyword or keyword in line.casefold():
                        lines.append(f'[{source.stem}] {line}')
            except OSError as exc:
                self.path.setText(f'{self.folder}\n日志读取失败：{exc}')
        if lines:
            scroll = self.text.verticalScrollBar()
            value = scroll.value()
            self.text.appendPlainText('\n'.join(lines))
            scroll.setValue(scroll.maximum() if self.follow.isChecked() else value)


class MonitorView(QWidget):
    def __init__(self):
        super().__init__()
        self.folder, self.sequence, self.latest = None, 0, None
        self.zero_stamp = None
        box = QVBoxLayout(self)
        bar = QHBoxLayout()
        self.pose_frame = QLabel('实时反馈 · base_link → tool_tcp')
        bar.addWidget(self.pose_frame)
        self.window = QComboBox()
        for seconds in (30, 60, 120):
            self.window.addItem(f'最近 {seconds} 秒', seconds)
        self.pause = QCheckBox('暂停曲线（继续记录）')
        bar.addWidget(self.window); bar.addWidget(self.pause)
        box.addLayout(bar)
        self.health = QLabel('等待会话反馈')
        self.health.setWordWrap(True)
        box.addWidget(self.health)
        self.tabs = QTabWidget()
        box.addWidget(self.tabs, 1)
        self.charts = {}
        pages = [
            ('末端与关节', [
                ('tcp_xyz_mm', '末端位置', ('X', 'Y', 'Z'), 'mm', False),
                ('tcp_rpy_deg', '末端姿态', ('Roll', 'Pitch', 'Yaw'), '°', True),
                ('joint_deg', '模型关节角度', tuple(f'J{i}' for i in range(1, 7)), '°', False),
                ('joint_deg_s', '模型关节速度', tuple(f'J{i}' for i in range(1, 7)), '°/s', False)]),
            ('七电机与夹爪', [
                ('motor_deg', '电机角度（按标定换算）', tuple(f'M{i}' for i in range(1, 8)), '°', False),
                ('motor_deg_s', '电机速度（按标定换算）', tuple(f'M{i}' for i in range(1, 8)), '°/s', False),
                ('gripper_mm', '夹爪单指行程', ('单指',), 'mm', False),
                ('gripper_mm_s', '夹爪单指速度', ('单指',), 'mm/s', False)])]
        for title, specs in pages:
            page = QWidget()
            grid = QGridLayout(page)
            for i, (key, name, labels, unit, wrap) in enumerate(specs):
                chart = StripChart(name, labels, unit, wrap)
                self.charts[key] = chart
                grid.addWidget(chart, i//2, i % 2)
            self.tabs.addTab(page, title)
        self.gripper_page = QWidget()
        gripper_box = QVBoxLayout(self.gripper_page)
        self.gripper_readout = QLabel('夹爪反馈开度：—　目标开度：—')
        self.gripper_readout.setWordWrap(True)
        gripper_box.addWidget(self.gripper_readout)
        opening_chart = StripChart('夹爪开合 · 目标与反馈', ('反馈开度', '目标开度'), '%', include_range=(0, 100))
        self.charts['gripper_open_percent'] = opening_chart
        gripper_box.addWidget(opening_chart, 1)
        self.gripper_limits = QLabel('夹爪 0%／100% 对应电子限位端点')
        self.gripper_limits.setWordWrap(True)
        gripper_box.addWidget(self.gripper_limits)
        gripper_note = QLabel('0% = 电子闭合端，100% = 电子张开端。青色为反馈，橙色为目标。\n'
                             '改变目标只改变橙色曲线；规划并执行后，青色曲线才随实际反馈变化。\n'
                             '反馈缺失时青色曲线断开；保留超出 0–100% 的反馈以便排查标定，不进行截断。')
        gripper_note.setWordWrap(True)
        gripper_box.addWidget(gripper_note)
        self.tabs.addTab(self.gripper_page, '夹爪开合')
        self.logs = LogViewer()
        self.tabs.addTab(self.logs, '运行日志')
        note = QLabel('20 Hz 反馈采样 / CSV 记录；曲线窗口 30–120 秒，纵轴自动缩放，不截断数值。\n'
                      'RPY 使用 Rz(Yaw)·Ry(Pitch)·Rx(Roll)，角度跨 ±180° 时断线；不是角速度。\n'
                      '夹爪电机 M7 角度按标定映射还原；单指毫米行程与电机角度分别显示。')
        note.setWordWrap(True)
        box.addWidget(note)
        self.window.currentIndexChanged.connect(self.set_window)

    def set_window(self):
        for chart in self.charts.values():
            chart.window_s = self.window.currentData()
            chart.update()

    def refresh(self, session, state):
        self.gripper_limits.setText(state.get('gripper_limit_status', '夹爪 0%／100% 对应电子限位端点'))
        folder = state.get('directory')
        if folder != self.folder:
            self.folder, self.sequence, self.latest = folder, 0, None
            self.zero_stamp = None
            self.logs.set_folder(folder)
            self.health.setText('等待会话反馈')
            for chart in self.charts.values():
                chart.samples.clear()
                chart.update()
        if not hasattr(session, 'telemetry_snapshot'):
            return
        packet = session.telemetry_snapshot(self.sequence)
        error = packet['error']
        if packet['dropped_events']:
            error += f" 日志队列溢出，遗漏 {packet['dropped_events']} 个事件；请检查磁盘性能。"
        samples = packet['samples']
        if samples:
            self.latest = samples[-1]
        if self.latest:
            latest = self.latest
            valid = sum(latest['valid'])
            mode = '模拟' if state.get('mock') else '真机'
            stamp = datetime.fromtimestamp(latest['time_unix'], ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S.%f')[:-3]
            age = latest['feedback_age_s']
            age_text = '无反馈' if age is None else f'{age*1000:.0f} ms'
            self.health.setText(f'{mode} · {state.get("role", "")} · 有效关节 {valid}/7 · 反馈年龄 {age_text} · 更新 {stamp}'
                                + (' · 会话已结束，保留曲线' if not state.get('running') else '') + ('\n'+error if error else ''))
            self.health.setStyleSheet('color:#ffbc70;' if error or valid != 7 else 'color:#9be28c;')
            relative, zero_stamp = latest.get('plot_relative', False), latest.get('plot_zero_at')
            self.pose_frame.setText('实时反馈 · 回零位姿为零参考' if relative and zero_stamp is not None else
                                    '实时反馈 · 等待回零建立零参考' if relative else '实时反馈 · base_link → tool_tcp')
            feedback, target = latest.get('gripper_open_percent', [None, None])
            feedback_text = '—（反馈缺失）' if feedback is None else f'{feedback:.1f}%'
            target_text = '—' if target is None else f'{target:.1f}%'
            motion = {'opening': '张开中', 'closing': '闭合中', 'stationary': '当前静止',
                      'unknown': '速度反馈缺失'}.get(latest.get('gripper_motion'), '等待速度反馈')
            self.gripper_readout.setText(f'夹爪反馈开度：{feedback_text}　目标开度：{target_text}　{motion}')
            if samples and not self.pause.isChecked():
                for sample in samples:
                    if sample.get('plot_zero_at') != self.zero_stamp:
                        for key in ('tcp_xyz_mm', 'tcp_rpy_deg'):
                            self.charts[key].samples.clear()
                        self.zero_stamp = sample.get('plot_zero_at')
                    for key, chart in self.charts.items():
                        chart.append(sample['elapsed_s'], sample[key])
                self.sequence = latest['sequence']
                for chart in self.charts.values():
                    if chart.isVisible():
                        chart.update()
        elif error:
            self.health.setText(error)
            self.health.setStyleSheet('color:#ffbc70;')
