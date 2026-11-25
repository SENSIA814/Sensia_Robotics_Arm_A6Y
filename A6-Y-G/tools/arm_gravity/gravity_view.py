#!/usr/bin/env python3
"""Native desktop gravity tuning UI; no devices are opened by constructing it."""
import argparse
from collections import deque
from copy import deepcopy
import math
from pathlib import Path
import signal
import sys
import time

from gravity_config import (JOINTS, ROLE_ORDER, VECTOR_KEYS, LIVE_KEYS, CAPS, PROFILE_DIR, load_roles, load_profile,
                            save_profile, base_parameters, sag_trial_parameters, validate_parameters, load_model, GravityModel)
from gravity_session import GravitySession
from scanner import inventory
from arm_view import STYLE
from PyQt5.QtCore import Qt, QTimer, QUrl, QPointF
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QScrollArea, QLayout, QLabel, QPushButton, QButtonGroup, QComboBox, QCheckBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QAbstractItemView, QDoubleSpinBox, QTabWidget)


class TorquePlot(QWidget):
    COLORS = ('#71a8ff','#67dfba','#ffbb7b')
    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=700)
        self.setMinimumHeight(150)
        self.last_sequence = -1
        self.joint = 1
    def append(self, state):
        if state.get('sequence',0) == self.last_sequence or not state.get('gravity'): return
        self.last_sequence = state['sequence']
        self.samples.append((time.monotonic(),state['gravity'],state['commanded'],state['effort']))
        self.update()
    def clear(self):
        self.samples.clear(); self.last_sequence = -1; self.update()
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#111d28'))
        left,top,w,h = 56,35,max(1,self.width()-74),max(1,self.height()-66)
        now = self.samples[-1][0] if self.samples else time.monotonic()
        values = [v for row in self.samples for series in row[1:] for v in [series[self.joint]]
                  if v is not None and math.isfinite(v)]
        scale = max(1.,max(map(abs,values),default=1.)*1.15)
        p.setPen(QColor('#52697e'))
        for i in range(5):
            y = top+i*h/4
            p.drawLine(QPointF(left,y),QPointF(left+w,y))
            p.drawText(2,int(y+4),f'{scale*(1-i/2):+.1f}')
        p.drawText(left,20,f'J{self.joint+1} 力矩 / N·m     最近 30 秒')
        p.drawText(left,top+h+22,'−30 s')
        p.drawText(left+w-36,top+h+22,'现在')
        for k,color in enumerate(self.COLORS):
            path = QPainterPath(); first = True
            for row in self.samples:
                if row[0] < now-30: continue
                value = row[k+1][self.joint]
                if value is None or not math.isfinite(value): first = True; continue
                point = QPointF(left+(row[0]-now+30)*w/30,top+h/2-value*h/(2*scale))
                if first: path.moveTo(point); first=False
                else: path.lineTo(point)
            p.setPen(QPen(QColor(color),2)); p.drawPath(path)
        p.end()


class GravityWindow(QMainWindow):
    def __init__(self, session=None, session_factory=None):
        super().__init__()
        self.roles = load_roles()
        self.reference_parameters = base_parameters()
        self.session_factory = session_factory or (lambda: GravitySession(self.roles))
        self.session = session or self.session_factory()
        self.sessions = {}
        self.drafts = {}
        self.saved_profiles = {}
        self.profile_notes = {}
        self.role_errors = {}
        self.plot_histories = {}
        self.plot_sequences = {}
        self.role = None
        self.saved_parameters = None
        self._closing = False
        self._teaching_close_pending = False
        self._close_queue = []
        self._closing_role = None
        self._close_faults_before = {}
        self.last_inventory = 0
        self.last_session_message = ''
        self.setWindowTitle('A6Y · 重力补偿实时调参终端')
        self.resize(1320,1000)
        self.setMinimumSize(1100,820)
        self.setStyleSheet(STYLE+'''QDoubleSpinBox { background:#192634; border:1px solid #34485a; padding:5px; }
QTabBar::tab { background:#192634; padding:10px 25px; } QTabBar::tab:selected { color:#67dfba; }
QTabWidget::pane { border:1px solid #34485a; }''')
        central = QWidget(); self.setCentralWidget(central)
        box = QVBoxLayout(central); box.setContentsMargins(22,16,22,16); box.setSpacing(10)
        label=QLabel('A6Y  /  GRAVITY COMPENSATION'); label.setObjectName('eyebrow'); box.addWidget(label)
        label=QLabel('重力补偿 · 选择机械臂与调试'); label.setObjectName('title'); box.addWidget(label)
        label=QLabel('沿用原 C++ / Pinocchio 控制器 · MIT 零刚度力矩模式 · 夹爪反馈参与模型，夹爪电机保持只读')
        label.setObjectName('note'); box.addWidget(label)
        self.teaching_button=QPushButton('打开动力示教 · 1 对 1 / 2 对 2')
        self.teaching_button.clicked.connect(self.open_teaching); box.addWidget(self.teaching_button)
        grid=QGridLayout(); box.addLayout(grid)
        self.group=QButtonGroup(self); self.buttons={}; self.batch_checks={}
        for i,role in enumerate(ROLE_ORDER):
            b=QPushButton(self.roles[role]['name']+'\nUSB '+self.roles[role]['usb_serial'])
            b.setCheckable(True); b.setMinimumHeight(88)
            b.clicked.connect(lambda _=False,r=role:self.select_role(r))
            self.group.addButton(b); self.buttons[role]=b; grid.addWidget(b,0,i)
            check=QCheckBox('加入批量操作'); check.toggled.connect(self.refresh)
            self.batch_checks[role]=check; grid.addWidget(check,1,i)
        row=QHBoxLayout(); box.addLayout(row)
        self.batch_status=QLabel('批量操作：先勾选机械臂'); row.addWidget(self.batch_status,1)
        self.batch_start=QPushButton('为勾选臂建立反馈'); self.batch_start.clicked.connect(self.start_selected); row.addWidget(self.batch_start)
        self.batch_enable=QPushButton('开启勾选臂补偿'); self.batch_enable.clicked.connect(self.enable_selected); row.addWidget(self.batch_enable)
        self.release_all=QPushButton('已托住全部 · 全部立即失能'); self.release_all.setObjectName('stop')
        self.release_all.clicked.connect(self.release_every_arm); row.addWidget(self.release_all)
        row=QHBoxLayout(); box.addLayout(row)
        self.mode=QComboBox(); self.mode.addItem('离线模拟（不连接任何机械臂）',True); self.mode.addItem('真机反馈与重力补偿',False)
        row.addWidget(self.mode,2)
        self.rviz=QCheckBox('打开 RViz 模型'); self.rviz.setChecked(True); row.addWidget(self.rviz)
        self.start_button=QPushButton('建立反馈连接'); self.start_button.clicked.connect(self.start); row.addWidget(self.start_button)
        self.disconnect_button=QPushButton('当前臂回零、失能并断开'); self.disconnect_button.clicked.connect(self.stop_current); row.addWidget(self.disconnect_button)
        self.status=QLabel('请选择机械臂；打开窗口不会连接电机。'); self.status.setObjectName('status'); self.status.setWordWrap(True); box.addWidget(self.status)
        self.error=QLabel(''); self.error.setObjectName('error'); self.error.setWordWrap(True); box.addWidget(self.error)
        row=QHBoxLayout(); box.addLayout(row)
        self.enable_button=QPushButton('开启重力补偿'); self.enable_button.setObjectName('start'); self.enable_button.clicked.connect(self.enable); row.addWidget(self.enable_button,1)
        self.release_button=QPushButton('已托住当前臂 · 立即失能（跳过回零）'); self.release_button.setObjectName('stop'); self.release_button.clicked.connect(self.release_current); row.addWidget(self.release_button,1)
        self.logs=QPushButton('本次记录'); self.logs.clicked.connect(self.open_logs); row.addWidget(self.logs)
        label=QLabel('点击上方机械臂卡片切换详情和调参，不会停止其他臂。连接只读反馈，另点开启才接管。关窗按臂逐一回零、失能；失败则停止退出，其余臂保持原状态。立即失能跳过回零，请先托住相应机械臂；失能不等于切断物理电源。')
        label.setWordWrap(True); label.setObjectName('note'); box.addWidget(label)
        self.tabs=QTabWidget(); box.addWidget(self.tabs,1)
        live=QWidget(); layout=QVBoxLayout(live); layout.setSizeConstraint(QLayout.SetMinimumSize)
        live_scroll=QScrollArea(); live_scroll.setWidgetResizable(True); live_scroll.setWidget(live)
        self.tabs.addTab(live_scroll,'实时反馈与曲线')
        self.table=QTableWidget(7,7)
        self.table.setHorizontalHeaderLabels(['关节','模型位置 ° / mm','速度 rad/s / m/s','模型重力 Nm / N','控制指令 Nm / N','反馈力矩 Nm / N','电机反馈'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().hide(); self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers); self.table.setFixedHeight(240)
        for i in range(7):
            self.table.setRowHeight(i,27)
            for j in range(7): self.table.setItem(i,j,QTableWidgetItem('—'))
        layout.addWidget(self.table)
        row=QHBoxLayout(); layout.addLayout(row)
        self.plot_joint=QComboBox()
        for i in range(6): self.plot_joint.addItem('J'+str(i+1),i)
        self.plot_joint.setCurrentIndex(1)
        self.plot=TorquePlot()
        self.plot_joint.currentIndexChanged.connect(self.change_joint)
        row.addWidget(self.plot_joint)
        row.addWidget(QLabel('蓝：模型重力    绿：控制器指令    橙：电机反馈    ·    只读阶段指令为 0'),1)
        layout.addWidget(self.plot,1)
        tuning=QWidget(); tl=QVBoxLayout(tuning); tl.setSizeConstraint(QLayout.SetMinimumSize)
        tuning_scroll=QScrollArea(); tuning_scroll.setWidgetResizable(True); tuning_scroll.setWidget(tuning)
        tuning_page=QWidget(); tuning_panel=QVBoxLayout(tuning_page); tuning_panel.setContentsMargins(0,0,0,0)
        tuning_panel.addWidget(tuning_scroll,1)
        self.tabs.addTab(tuning_page,'实时调参 / 固定参数')
        note=QLabel('参考：交接包 a5_gravity_controllers.yaml。逐轴重力系数直接生效，不再乘标量 1.5。四臂按 USB 独立保存；方向使用模型坐标。')
        note.setWordWrap(True); tl.addWidget(note)
        self.parameter_table=QTableWidget(6,7)
        self.parameter_table.setMinimumHeight(325)
        self.parameter_table.setHorizontalHeaderLabels(['关节','重力系数','补偿方向','阻尼','总力矩上限 Nm','摩擦补偿 Nm','已确认系数 / 阻尼 / 摩擦'])
        self.parameter_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch); self.parameter_table.verticalHeader().hide()
        self.parameter_table.horizontalHeader().setSectionResizeMode(6,QHeaderView.ResizeToContents)
        self.parameter_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.editors={k:[] for k in VECTOR_KEYS}
        for i in range(6):
            self.parameter_table.setItem(i,0,QTableWidgetItem('J'+str(i+1)))
            self.parameter_table.setItem(i,6,QTableWidgetItem('—'))
            self.parameter_table.setRowHeight(i,46)
            for c,k in enumerate(VECTOR_KEYS,1):
                if k == 'effort_signs':
                    e=QComboBox(); e.addItem('+1',1.); e.addItem('−1',-1.)
                else:
                    e=QDoubleSpinBox(); e.setDecimals(3); e.setSingleStep(.005 if k == 'damping_gains' else .05)
                    e.setRange(0,3 if k == 'torque_scales' else 5 if k == 'damping_gains' else CAPS[i])
                self.editors[k].append(e); self.parameter_table.setCellWidget(i,c,e)
        tl.addWidget(self.parameter_table)
        self.friction=QCheckBox('启用原程序的连续摩擦补偿'); tl.addWidget(self.friction)
        note=QLabel('摩擦助力沿运动方向输出，不能区分手动拖动和自行下垂。先调重力平衡，再调摩擦助力；关闭时表内摩擦幅值保留但不输出。')
        note.setWordWrap(True); note.setObjectName('note'); tl.addWidget(note)
        self.parameter_summary=QLabel(); self.parameter_summary.setWordWrap(True); tl.addWidget(self.parameter_summary)
        self.tuning_status=QLabel('先选择机械臂。'); self.tuning_status.setWordWrap(True); tuning_panel.addWidget(self.tuning_status)
        for key,editors in self.editors.items():
            for i,e in enumerate(editors):
                e.setToolTip(f'交接参考值：{self.reference_parameters[key][i]:g}')
                (e.currentIndexChanged if key == 'effort_signs' else e.valueChanged).connect(self.update_parameter_summary)
        self.friction.toggled.connect(self.update_parameter_summary)
        row=QHBoxLayout(); tuning_panel.addLayout(row)
        self.apply=QPushButton('实时应用（0.3 秒）'); self.apply.setObjectName('start'); self.apply.clicked.connect(self.apply_parameters); row.addWidget(self.apply)
        self.save=QPushButton('满意，固定当前参数'); self.save.clicked.connect(self.save_parameters); row.addWidget(self.save)
        self.reset=QPushButton('恢复交接参考值'); self.reset.clicked.connect(self.restore_reference); row.addWidget(self.reset)
        self.trial=QPushButton('载入下垂试调起点'); self.trial.clicked.connect(self.load_sag_trial); row.addWidget(self.trial)
        tl.addStretch()
        self.profile_note=QLabel('参数未选择'); self.profile_note.setObjectName('note'); self.profile_note.setWordWrap(True); box.addWidget(self.profile_note)
        self.set_parameters(base_parameters())
        self.timer=QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(100)
        self.refresh()

    def open_teaching(self):
        if any(s.snapshot()['running'] for s in self.sessions.values()):
            self.error.setText('请先回零、失能并断开调参会话，再启动动力示教。'); self.error.show(); return
        if not hasattr(self,'teaching_window'):
            from teaching_view import TeachingWindow
            self.teaching_window=TeachingWindow()
        self.teaching_window.show();self.teaching_window.raise_();self.teaching_window.activateWindow()

    def select_role(self, role):
        try:
            if self.role == role:
                self.buttons[role].setChecked(True)
                return
            if self.role:
                self.drafts[self.role]=self.parameters()
                self.saved_profiles[self.role]=deepcopy(self.saved_parameters)
                self.profile_notes[self.role]=self.profile_note.text()
                self.role_errors[self.role]=self.error.text()
            if role not in self.sessions:
                self.sessions[role]=self.session if self.role is None else self.session_factory()
            self.role=role; self.buttons[role].setChecked(True)
            self.session=self.sessions[role]
            if role not in self.drafts:
                self.drafts[role]=load_profile(role,self.roles)
                exists=(PROFILE_DIR/(role+'.yaml')).exists()
                self.saved_profiles[role]=deepcopy(self.drafts[role]) if exists else None
                self.profile_notes[role]=self.roles[role]['name']+' · USB '+self.roles[role]['usb_serial']+' · '+('已载入该臂保存参数' if exists else '已载入交接参考参数')
            self.set_parameters(self.drafts[role])
            self.saved_parameters=deepcopy(self.saved_profiles[role])
            self.profile_note.setText(self.profile_notes[role])
            self.error.setText(self.role_errors.get(role,''))
            self.last_session_message=''
            self.plot.samples=self.plot_histories.setdefault(role,deque(maxlen=700))
            self.plot.last_sequence=self.plot_sequences.get(role,-1); self.plot.update()
            for i in range(7):
                for j in range(7): self.table.item(i,j).setText('—')
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def set_parameters(self,p):
        for k,editors in self.editors.items():
            for i,e in enumerate(editors):
                if k == 'effort_signs': e.setCurrentIndex(0 if p[k][i] == 1 else 1)
                else: e.setValue(p[k][i])
        self.friction.setChecked(p['friction_compensation_enabled'])
        self.update_parameter_summary()

    def update_parameter_summary(self, *_):
        reference=self.reference_parameters; changes=[]
        labels={'torque_scales':'重力系数','effort_signs':'方向','damping_gains':'阻尼',
                'torque_limits':'限幅','friction_compensations':'摩擦幅值'}
        for key,editors in self.editors.items():
            for i,e in enumerate(editors):
                value=e.currentData() if key == 'effort_signs' else e.value()
                if value != reference[key][i]:
                    changes.append(f'J{i+1} {labels[key]} {reference[key][i]:g} → {value:g}')
        if self.friction.isChecked() != reference['friction_compensation_enabled']:
            changes.append('连续摩擦助力'+('开启' if self.friction.isChecked() else '关闭'))
        self.parameter_summary.setText('当前值相对交接参考：'+('；'.join(changes) if changes else '六轴参数一致'))

    def restore_reference(self):
        self.load_draft(base_parameters())
        self.profile_note.setText('已载入交接参考值。连接期间需点击“实时应用”；满意后再固定。')

    def load_draft(self,p):
        if self.session.snapshot()['running']:
            current=self.parameters()
            for key in LIVE_KEYS: current[key]=p[key]
            p=current
        self.set_parameters(p)
        self.refresh()

    def load_sag_trial(self):
        self.load_draft(sag_trial_parameters())
        self.profile_note.setText('未标定试调起点：J2 / J3 / J4 = 1.2 / 1.1 / 1.2，摩擦助力关闭。尚未自动应用或保存。')

    def apply_parameters(self):
        try: self.session.tune(self.parameters()); self.error.clear()
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def parameters(self):
        p=deepcopy(self.reference_parameters)
        for k,editors in self.editors.items():
            p[k][:6]=[e.currentData() if k == 'effort_signs' else e.value() for e in editors]
        p['friction_compensation_enabled']=self.friction.isChecked()
        return validate_parameters(p)

    def save_parameters(self):
        try:
            p=self.parameters(); state=self.session.snapshot()
            if state['running'] and (state['busy'] or p != state.get('applied_parameters')):
                raise ValueError('请先实时应用并等待确认，再固定参数')
            path=save_profile(self.role,p,self.roles)
            self.saved_parameters=p
            self.saved_profiles[self.role]=deepcopy(p)
            self.profile_note.setText('已保存：'+str(path)); self.error.clear()
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def start(self):
        try:
            if self._closing: raise ValueError('正在逐臂退出，请等待或先立即失能')
            self.session.start(self.role,mock=self.mode.currentData(),with_rviz=self.rviz.isChecked(),parameters=self.parameters())
            self.plot.clear(); self.plot_sequences[self.role]=-1; self.error.clear()
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def enable(self):
        try:
            if self._closing: raise ValueError('正在逐臂退出，请等待或先立即失能')
            self.session.enable(); self.error.clear()
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def selected_roles(self):
        return [r for r in ROLE_ORDER if self.batch_checks[r].isChecked()]

    def start_selected(self):
        if self._closing: return
        errors=[]
        for role in self.selected_roles():
            try:
                if role not in self.sessions:
                    self.sessions[role]=self.session if not self.sessions else self.session_factory()
                session=self.sessions[role]
                if session.snapshot()['running']: continue
                p=self.parameters() if role==self.role else self.drafts.get(role)
                if p is None:
                    p=load_profile(role,self.roles)
                    self.drafts[role]=deepcopy(p)
                    exists=(PROFILE_DIR/(role+'.yaml')).exists()
                    self.saved_profiles[role]=deepcopy(p) if exists else None
                    self.profile_notes[role]=self.roles[role]['name']+' · USB '+self.roles[role]['usb_serial']+' · '+('已载入该臂保存参数' if exists else '已载入交接参考参数')
                session.start(role,mock=self.mode.currentData(),with_rviz=self.rviz.isChecked(),parameters=p)
                self.plot_histories.setdefault(role,deque(maxlen=700)).clear()
                self.plot_sequences[role]=-1
            except Exception as exc: errors.append(self.roles[role]['name']+'：'+str(exc))
        if self.role is None and self.selected_roles(): self.select_role(self.selected_roles()[0])
        self.error.setText('；'.join(errors))
        self.refresh()

    def enable_selected(self):
        if self._closing: return
        targets=[]; errors=[]
        for role in self.selected_roles():
            session=self.sessions.get(role)
            state=session.snapshot() if session else {}
            if state.get('active'): continue
            if not state.get('ready') or state.get('busy') or state.get('closing_requested'):
                errors.append(self.roles[role]['name']+'：请先连接反馈并等待就绪')
            else:
                reason=session.enable_block_reason()
                if reason: errors.append(self.roles[role]['name']+'：'+reason)
                else: targets.append((role,session))
        # Validate all selected arms before sending any enable request.
        if not errors:
            for role,session in targets:
                try: session.enable()
                except Exception as exc: errors.append(self.roles[role]['name']+'：'+str(exc))
        self.error.setText('；'.join(errors)); self.refresh()

    def stop_current(self):
        try:
            if self._closing: return
            if any(r!=self.role and s.snapshot().get('closing_requested') and s.snapshot()['running']
                   for r,s in self.sessions.items()):
                raise ValueError('另一条臂正在回零退出，请等待其完成')
            self.session.stop()
        except Exception as exc: self.error.setText(str(exc))
        self.refresh()

    def release_current(self):
        self.session.release(); self.refresh()

    def release_every_arm(self):
        self._closing=False; self._close_queue=[]; self._closing_role=None
        errors=[]
        for role,session in self.sessions.items():
            if session.snapshot()['running']:
                try: session.release()
                except Exception as exc: errors.append(self.roles[role]['name']+'：'+str(exc))
        self.error.setText('；'.join(errors)); self.refresh()

    @staticmethod
    def session_alive(session):
        return session.snapshot()['running'] or bool(session.worker and session.worker.is_alive())

    def advance_close(self):
        if not self._closing: return
        for role,session in self.sessions.items():
            if role==self._closing_role: continue
            state=session.snapshot()
            fault=(state.get('error',''),state.get('notice',''),bool(state.get('close_failed')))
            if (state.get('error') or state.get('close_failed')) and fault!=self._close_faults_before.get(role):
                self._closing=False; self._close_queue=[]; self._closing_role=None
                self.select_role(role)
                self.error.setText(self.roles[role]['name']+'出现异常，已停止后续回零；已开始的回零继续完成，其余臂保持原状态。'+
                                   (state.get('error') or state.get('notice') or '失能状态未确认'))
                return
        if self._closing_role:
            session=self.sessions[self._closing_role]; state=session.snapshot()
            if state.get('close_failed') or (not self.session_alive(session) and state.get('release_confirmed') is False):
                role=self._closing_role
                self._closing=False; self._close_queue=[]; self._closing_role=None
                self.select_role(role)
                self.error.setText(self.roles[role]['name']+'退出未完成，已停止后续回零；其余臂保持原状态。'+
                                   (state.get('notice') or state.get('error') or '失能状态未确认'))
                return
            if self.session_alive(session): return
            self._closing_role=None
        while self._close_queue:
            role=self._close_queue.pop(0); session=self.sessions[role]
            if not self.session_alive(session): continue
            self._closing_role=role
            try: session.stop()
            except Exception as exc:
                self._closing=False; self._close_queue=[]; self._closing_role=None
                self.select_role(role); self.error.setText(str(exc))
            return
        self._closing=False
        self.close()

    def open_logs(self):
        directory=self.session.snapshot().get('directory')
        if directory: QDesktopServices.openUrl(QUrl.fromLocalFile(directory))

    def change_joint(self):
        self.plot.joint=self.plot_joint.currentData(); self.plot.update()

    def refresh(self):
        states={r:session.snapshot() for r,session in self.sessions.items()}
        any_running=any(state['running'] for state in states.values())
        for role,button in self.buttons.items():
            state=states.get(role,{})
            tag=('回零 / 退出失败 · 查看详情' if state.get('close_failed') else
                 '回零 / 退出中' if state.get('closing_requested') and state.get('running') else
                 '异常 · 查看详情' if state.get('error') else
                 '正在处理' if state.get('busy') else
                 '补偿运行' if state.get('active') else
                 '反馈就绪 · 未开启补偿' if state.get('ready') else
                 '正在连接' if state.get('running') else '未连接')
            button.setText(self.roles[role]['name']+'\nUSB '+self.roles[role]['usb_serial']+'\n'+tag)
            button.setEnabled(True)
            if state.get('running') or state.get('error') or state.get('close_failed'):
                button.setToolTip((state.get('status') or '')+'\n'+(state.get('notice') or state.get('error') or ''))
            if state.get('gravity') and state.get('sequence',0)!=self.plot_sequences.get(role,-1):
                self.plot_sequences[role]=state.get('sequence',0)
                self.plot_histories.setdefault(role,deque(maxlen=700)).append(
                    (time.monotonic(),state['gravity'],state['commanded'],state['effort']))
                if role==self.role: self.plot.update()
        selected=self.selected_roles()
        self.batch_status.setText(('正在逐臂退出：'+self.roles[self._closing_role]['name'] if self._closing and self._closing_role else
                                   f'已勾选 {len(selected)} 条 · 反馈 {sum(s["ready"] for s in states.values())} 条 · 补偿 {sum(s["active"] for s in states.values())} 条'))
        self.batch_start.setEnabled(not self._closing and any(not states.get(r,{}).get('running') for r in selected))
        self.batch_enable.setEnabled(not self._closing and bool(selected) and
                                     all(states.get(r,{}).get('ready') and not states[r].get('busy') and
                                         not states[r].get('closing_requested') for r in selected) and
                                     any(not states[r].get('active') for r in selected))
        self.release_all.setEnabled(any_running)
        for check in self.batch_checks.values(): check.setEnabled(not self._closing)
        s=self.session.snapshot(); running=s['running']; busy=s['busy']
        self.mode.setEnabled(not any_running); self.rviz.setEnabled(not any_running)
        self.start_button.setEnabled(bool(self.role) and not running and not self._closing)
        other_closing=any(r!=self.role and v.get('closing_requested') and v['running'] for r,v in states.items())
        self.disconnect_button.setEnabled(running and not s.get('closing_requested') and not self._closing and not other_closing)
        blocked=self.session.enable_block_reason() if s['ready'] and not s['active'] and not busy else ''
        self.enable_button.setEnabled(s['ready'] and not busy and not s['active'] and not self._closing)
        self.enable_button.setToolTip(blocked)
        self.release_button.setEnabled(running)
        self.parameter_table.setEnabled(not busy and not self._closing)
        for key,editors in self.editors.items():
            for e in editors: e.setEnabled(not busy and (not running or key in LIVE_KEYS))
        self.friction.setEnabled(not busy and not self._closing)
        try: draft=self.parameters()
        except ValueError: draft=None
        applied=s.get('applied_parameters'); dirty=draft != applied
        self.apply.setEnabled(s['ready'] and not busy and not self._closing and draft is not None and dirty)
        self.save.setEnabled(bool(self.role) and not busy and not self._closing and draft is not None and (not running or not dirty))
        self.reset.setEnabled(not busy and not self._closing); self.trial.setEnabled(not busy and not self._closing)
        if s.get('tuning_error'): self.error.setText(s['tuning_error'])
        text=('参数无效，请调整' if draft is None else
              '正在平滑应用；右侧显示上次确认值' if s.get('tuning_pending') else
              '有编辑值尚未应用，点击“实时应用”' if running and dirty else
              s.get('tuning_status','已准备') if running else '未连接：编辑值将在建立连接时载入')
        text+=' · '+('已固定，下次自动加载' if draft is not None and draft == self.saved_parameters else '尚未固定')
        self.tuning_status.setText(text)
        for i in range(6):
            values=([applied['torque_scales'][i],applied['damping_gains'][i],
                     applied['friction_compensations'][i] if applied['friction_compensation_enabled'] else 0.] if applied and running else None)
            self.parameter_table.item(i,6).setText(' / '.join(f'{v:.3f}' for v in values) if values else '—')
        self.logs.setEnabled(bool(s.get('directory')))
        mode='离线模拟' if (s['mock'] if running else self.mode.currentData()) else '真机'
        self.status.setText((self.roles[self.role]['name']+' · ' if self.role else '')+mode+' · '+s['status']+('；'+blocked if blocked else ''))
        session_message=s.get('error') or s.get('notice') or s.get('tuning_error') or ''
        if session_message: self.error.setText(session_message)
        elif self.last_session_message: self.error.clear()
        self.last_session_message=session_message
        self.error.setVisible(bool(self.error.text()))
        if not any_running and time.monotonic()-self.last_inventory > 2:
            self.last_inventory=time.monotonic()
            for arm in inventory(self.roles):
                self.buttons[arm['role']].setToolTip(arm.get('connection_message','')+'\n'+arm['port'])
        hw=s.get('hardware',{}); q=hw.get('q'); velocity=hw.get('velocity')
        if q is not None:
            for i,n in enumerate(JOINTS):
                row=hw.get('rows',{}).get(n,{})
                values=['J'+str(i+1) if i < 6 else '夹爪（只读）',q[i]*(180/math.pi if i < 6 else 1000),
                        velocity[i] if velocity else None,
                        s.get('gravity',[None]*7)[i] if s.get('gravity') else None,
                        s.get('commanded',[None]*7)[i] if s.get('commanded') else None,
                        s.get('effort',[None]*7)[i] if s.get('effort') else None,
                        '模拟' if s['mock'] else ('过期 / 故障' if not s.get('fresh',False) else
                           '使能' if row.get('motor_error') == 1 else '失能')]
                for j,v in enumerate(values):
                    text=v if isinstance(v,str) else ('—' if v is None or not math.isfinite(v) else f'{v:+.4f}')
                    self.table.item(i,j).setText(text)
        self.advance_close()

    def closeEvent(self,event):
        if hasattr(self,'teaching_window'):
            teaching=self.teaching_window.coordinator
            if teaching.alive() or teaching.snapshot()['running'] or (teaching.worker and teaching.worker.is_alive()):
                self._teaching_close_pending=True
                self.teaching_window.close()
                event.ignore()
                QTimer.singleShot(200,self.finish_teaching_close)
                return
        alive=[r for r in ROLE_ORDER if r in self.sessions and self.session_alive(self.sessions[r])]
        if alive:
            if not self._closing:
                # Finish any individually requested return before starting another.
                self._close_queue=sorted(alive,key=lambda r:not self.sessions[r].snapshot().get('closing_requested',False))
                self._close_faults_before={r:(s.snapshot().get('error',''),s.snapshot().get('notice',''),bool(s.snapshot().get('close_failed')))
                                           for r,s in self.sessions.items()}
                self._closing_role=None; self._closing=True
                self.advance_close()
            event.ignore()
        else: event.accept()

    def finish_teaching_close(self):
        if not self._teaching_close_pending:return
        teaching=self.teaching_window.coordinator
        if teaching.snapshot()['close_failed']:
            self._teaching_close_pending=False
            self.error.setText('动力示教退出保护未完成，请在示教窗口处理后重试。');self.error.show()
        elif teaching.alive() or teaching.snapshot()['running'] or (teaching.worker and teaching.worker.is_alive()):
            QTimer.singleShot(200,self.finish_teaching_close)
        else:
            self._teaching_close_pending=False;self.close()


def main():
    parser=argparse.ArgumentParser(description='A6Y 四臂重力补偿调试窗口')
    parser.add_argument('--check',action='store_true',help='检查四臂模型、身份配置和参数，不连接硬件')
    parser.add_argument('--role',choices=ROLE_ORDER)
    parser.add_argument('--no-rviz',action='store_true')
    args=parser.parse_args()
    if args.check:
        roles=load_roles()
        for role in ROLE_ORDER:
            p=load_profile(role,roles); model=load_model(role,roles)
            g=GravityModel(model).gravity([0.,1.2,1.2,0.,0.,0.,-.01])
            print(role,roles[role]['usb_serial'],'g(q) =',g.round(5).tolist())
        print('四臂模型与参数检查通过；没有连接硬件。')
        return
    app=QApplication(sys.argv[:1]); app.setFont(QFont('Noto Sans CJK SC',10))
    window=GravityWindow(); window.rviz.setChecked(not args.no_rviz)
    if args.role: window.select_role(args.role)
    signal.signal(signal.SIGINT,lambda *_:window.close())
    signal.signal(signal.SIGTERM,lambda *_:window.close())
    window.show(); app.exec_()


if __name__ == '__main__': main()
