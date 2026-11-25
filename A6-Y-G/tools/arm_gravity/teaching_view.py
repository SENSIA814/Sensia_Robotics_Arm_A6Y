#!/usr/bin/env python3
"""Desktop pair selection and protected lifecycle for joint teaching."""
import argparse
import json
from pathlib import Path
import signal
import sys
import threading
import time
import traceback

from PyQt5.QtCore import Qt,QTimer,QUrl
from PyQt5.QtGui import QFont,QDesktopServices
from PyQt5.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,
    QComboBox,QCheckBox,QDoubleSpinBox,QPushButton,QTableWidget,QTableWidgetItem,QHeaderView,
    QPlainTextEdit,QFileDialog,QSplitter)

from teaching import DEFAULT_PAIRS, validate_pairs
from teaching_port import TeachingCoordinator
from gravity_config import load_roles, load_model
from teaching_port_config import port_parameters, CONTROL_HZ, TARGET_HZ, CAN_BITRATE
from arm_view import STYLE
from teaching_logs import LogTail,export_report,summary_text


class TeachingWindow(QWidget):
    def __init__(self,coordinator=None):
        super().__init__();self.coordinator=coordinator or TeachingCoordinator()
        self.close_pending=False;self.setWindowTitle('A6Y · 动力示教与诊断');self.resize(1180,940)
        self.setStyleSheet(STYLE+'''
            QDoubleSpinBox {background:#192634; border:1px solid #34485a; padding:8px;}
            QPlainTextEdit {background:#09111a; color:#c8dae8; border:1px solid #2b3d50; font-size:12px;}
            QCheckBox {spacing:8px;} QCheckBox::indicator {width:16px; height:16px;}
            QSplitter::handle {background:#273a4c; height:3px;}
            QComboBox QAbstractItemView {background:#192634; color:#e0e9f2; selection-background-color:#244b42;}
            QPushButton#stop:disabled {color:#667a8e;}
        ''')
        self.log_folder=None;self.log_state=None;self.live_directory=None;self.log_reader=LogTail()
        self.export_worker=None;self.export_result=None;self.next_log_refresh=0.
        box=QVBoxLayout(self);box.setContentsMargins(22,18,22,18);box.setSpacing(10)
        title=QLabel('动力示教 · 原程序位置跟随');title.setStyleSheet('font-size:24px;font-weight:600');box.addWidget(title)
        note=QLabel('选择配对 → 示教臂开启重力补偿 → 运动臂跟随示教臂当前位置（启动不回零）\n'
                    '默认左对左、右对右；取消一组即为 1 对 1。仅 J1～J6 参与，夹爪保持只读。\n'
                    '阶跃采集：开始跟随后静止 2 秒 → 手动移动一次 → 保持至少 10 秒 → 停止并导出日志。')
        note.setWordWrap(True);box.addWidget(note)
        self.pair_checks=[];self.leaders=[];self.followers=[];self.configuration=[]
        for index,(leader,follower) in enumerate(DEFAULT_PAIRS):
            row=QHBoxLayout();box.addLayout(row)
            check=QCheckBox(f'启用第 {index+1} 组');check.setChecked(True);row.addWidget(check)
            source=QComboBox();target=QComboBox()
            for role in ('teaching_left','teaching_right'):source.addItem(self.coordinator.roles[role]['name'],role)
            for role in ('motion_left','motion_right'):target.addItem(self.coordinator.roles[role]['name'],role)
            source.setCurrentIndex(index);target.setCurrentIndex(index)
            row.addWidget(source,1);row.addWidget(QLabel('→'));row.addWidget(target,1)
            self.pair_checks.append(check);self.leaders.append(source);self.followers.append(target)
            self.configuration.extend((check,source,target))
        settings=QHBoxLayout();box.addLayout(settings)
        self.mode=QComboBox();self.mode.addItem('离线模拟',True);self.mode.addItem('真机动力示教',False)
        self.frequency=QDoubleSpinBox();self.frequency.setRange(TARGET_HZ,TARGET_HZ);self.frequency.setDecimals(0);self.frequency.setValue(TARGET_HZ);self.frequency.setSuffix(' Hz（固定）')
        self.rviz=QCheckBox('打开所选臂 RViz')
        settings.addWidget(self.mode);settings.addWidget(QLabel('目标发送频率'));settings.addWidget(self.frequency);settings.addWidget(self.rviz);settings.addStretch()
        self.configuration.extend((self.mode,self.frequency,self.rviz))
        box.addWidget(QLabel(f'底层控制 {CONTROL_HZ} Hz · 每轴 CAN 指令 {CONTROL_HZ} Hz · CAN {CAN_BITRATE/1e6:g} Mbps（配置值）'))
        self.status=QLabel('选择配对后启动');self.status.setObjectName('status');self.status.setWordWrap(True);box.addWidget(self.status)
        self.error=QLabel();self.error.setObjectName('error');self.error.setWordWrap(True)
        self.error.setTextInteractionFlags(Qt.TextSelectableByMouse);box.addWidget(self.error)
        actions=QHBoxLayout();box.addLayout(actions)
        self.start_button=QPushButton('启动：直接跟随示教臂');self.start_button.clicked.connect(self.start)
        self.start_button.setObjectName('start')
        self.stop_button=QPushButton('停止示教 · 回零并失能');self.stop_button.clicked.connect(self.coordinator.stop)
        self.release_button=QPushButton('已托住全部 · 立即失能');self.release_button.clicked.connect(self.release)
        self.release_button.setObjectName('stop')
        for button in (self.start_button,self.stop_button,self.release_button):actions.addWidget(button)
        self.table=QTableWidget(4,4);self.table.setHorizontalHeaderLabels(['机械臂','运行状态','J1～J6 反馈 / rad','失能核验'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for col in (0,2,3):self.table.horizontalHeader().setSectionResizeMode(col,QHeaderView.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers);self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().hide();self.table.setMinimumHeight(180)
        splitter=QSplitter(Qt.Vertical);splitter.addWidget(self.table);box.addWidget(splitter,1)
        logs=QWidget();logs_box=QVBoxLayout(logs);logs_box.setContentsMargins(0,5,0,0)
        log_actions=QHBoxLayout();logs_box.addLayout(log_actions)
        log_actions.addWidget(QLabel('运行日志'))
        self.log_scope=QComboBox();self.log_scope.addItem('全部机械臂与整组','all');self.log_scope.addItem('仅整组流程','coordinator')
        for role in ('teaching_left','motion_left','teaching_right','motion_right'):
            self.log_scope.addItem(self.coordinator.roles[role]['name'],role)
        self.log_scope.currentIndexChanged.connect(self.reset_log_view);log_actions.addWidget(self.log_scope)
        self.log_follow=QCheckBox('自动滚动');self.log_follow.setChecked(True);log_actions.addWidget(self.log_follow)
        self.log_detail=QCheckBox('详细输出');self.log_detail.toggled.connect(self.reset_log_view);log_actions.addWidget(self.log_detail)
        log_actions.addStretch()
        self.history_button=QPushButton('历史日志');self.history_button.clicked.connect(self.open_history)
        self.current_button=QPushButton('本次日志');self.current_button.clicked.connect(self.open_current)
        self.folder_button=QPushButton('打开目录');self.folder_button.clicked.connect(self.open_folder)
        self.copy_button=QPushButton('复制诊断摘要');self.copy_button.clicked.connect(self.copy_summary)
        self.export_button=QPushButton('导出诊断包');self.export_button.clicked.connect(self.export_logs)
        for button in (self.history_button,self.current_button,self.folder_button,self.copy_button,self.export_button):log_actions.addWidget(button)
        self.log_source=QLabel('日志将在启动后自动记录，也可打开历史记录');self.log_source.setObjectName('note')
        self.log_source.setWordWrap(True);logs_box.addWidget(self.log_source)
        self.step_status=QLabel('阶跃日志将在跟随开始后自动采集');self.step_status.setWordWrap(True)
        self.step_status.setObjectName('note');logs_box.addWidget(self.step_status)
        self.log_view=QPlainTextEdit();self.log_view.setReadOnly(True);self.log_view.setMaximumBlockCount(2000)
        self.log_view.setLineWrapMode(QPlainTextEdit.NoWrap);self.log_view.setMinimumHeight(130)
        self.log_view.setFont(QFont('monospace',10));logs_box.addWidget(self.log_view,1)
        self.log_notice=QLabel('逐轴采集目标、实际角度、误差与时间戳；停止后生成阶跃响应报告，随诊断包导出。')
        self.log_notice.setObjectName('note');self.log_notice.setWordWrap(True);logs_box.addWidget(self.log_notice)
        splitter.addWidget(logs);splitter.setSizes([190,260])
        self.metrics=QLabel();self.metrics.setWordWrap(True);box.addWidget(self.metrics)
        footer=QLabel('反馈速度范围按各电机实读 VMAX；逐轴数值与参数来源见启动日志。\n'
                      '使用原程序的目标映射与 POS_VEL 跟随；启动不回零；退出各臂同时原生位置回零，超时停止等待。跟随时无跨臂或环境碰撞模型，请保持运动空间畅通。\n'
                      '关窗 / Ctrl+C 执行正常退出保护；失败保留窗口。立即失能会跳过回零并失去力矩支撑。')
        footer.setWordWrap(True);box.addWidget(footer)
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100);self.refresh()
        previous=sorted(self.coordinator.report_dir.glob('teaching_*/result.json'),key=lambda p:p.stat().st_mtime,reverse=True)
        if previous and self.log_folder is None:self.select_log(previous[0].parent)

    def selected_log_state(self):
        state=self.coordinator.snapshot()
        return state if self.log_folder and str(self.log_folder)==state.get('directory') else self.log_state or {}

    def select_log(self,folder):
        try:
            folder=Path(folder)
            self.log_state=json.loads((folder/'result.json').read_text()) if (folder/'result.json').is_file() else {}
            if not (folder/'session.json').is_file() and not self.log_state:raise ValueError('请选择包含 session.json 或 result.json 的示教会话目录')
            self.log_folder=folder;self.log_source.setText('正在查看：'+str(folder));self.reset_log_view()
        except Exception as exc:self.log_notice.setText('打开日志失败：'+str(exc))

    def reset_log_view(self):
        self.log_reader=LogTail();self.log_view.clear();self.next_log_refresh=0.

    def open_history(self):
        folder=QFileDialog.getExistingDirectory(self,'选择示教会话日志目录',str(self.coordinator.report_dir))
        if folder:self.select_log(folder)

    def open_current(self):
        folder=self.coordinator.snapshot().get('directory')
        if folder:self.select_log(folder)

    def open_folder(self):
        if self.log_folder:QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.log_folder)))

    def copy_summary(self):
        QApplication.clipboard().setText(summary_text(self.selected_log_state()))
        self.log_notice.setText('诊断摘要已复制；完整数据请使用“导出诊断包”。')

    def export_logs(self):
        if not self.log_folder or (self.export_worker and self.export_worker.is_alive()):return
        filename,_=QFileDialog.getSaveFileName(self,'导出完整诊断包',str(self.log_folder.parent/(self.log_folder.name+'.zip')),'ZIP 诊断包 (*.zip)')
        if not filename:return
        if not filename.lower().endswith('.zip'):filename+='.zip'
        folder,state=self.log_folder,self.selected_log_state()
        self.export_result=None;self.export_button.setEnabled(False);self.log_notice.setText('正在打包日志…')
        def run():
            try:self.export_result='诊断包已保存：'+str(export_report(folder,filename,state))
            except Exception as exc:self.export_result='导出失败：'+str(exc)
        self.export_worker=threading.Thread(target=run,daemon=False);self.export_worker.start()

    def refresh_logs(self,state):
        folder=state.get('directory')
        if folder and folder!=self.live_directory and Path(folder,'session.json').is_file():
            self.live_directory=folder;self.select_log(folder)
        exporting=bool(self.export_worker and self.export_worker.is_alive())
        for button in (self.folder_button,self.copy_button):button.setEnabled(self.log_folder is not None)
        self.export_button.setEnabled(self.log_folder is not None and not exporting)
        self.current_button.setEnabled(bool(folder))
        if self.export_result is not None and not exporting:
            self.log_notice.setText(self.export_result);self.export_result=None
        if not self.log_folder or time.monotonic()<self.next_log_refresh:return
        self.next_log_refresh=time.monotonic()+.5
        try:
            quality_path=self.log_folder/'step_quality.json'
            if quality_path.is_file():
                quality=json.loads(quality_path.read_text())
                dropped=sum(quality.get('queue_dropped',{}).values())
                skipped=sum(s.get('skipped',0) for s in quality.get('cycles',{}).values())
                stage=('报告已生成：step_summary.txt' if quality.get('analysis_complete') else
                       '采集完成，正在生成报告' if quality.get('complete') else '正在采集 / 尚未完整关闭')
                if quality.get('error'):stage='记录异常：'+quality['error']
                self.step_status.setText(f'阶跃日志：{stage}；队列丢弃 {dropped}，未观测周期 {skipped}')
            else:self.step_status.setText('该会话暂无逐轴阶跃日志')
            lines=self.log_reader.read(self.log_folder,self.log_scope.currentData(),self.selected_log_state(),detailed=self.log_detail.isChecked())
            if lines:
                scroll=self.log_view.verticalScrollBar();old=scroll.value()
                self.log_view.appendPlainText('\n'.join(lines))
                scroll.setValue(scroll.maximum() if self.log_follow.isChecked() else old)
        except Exception as exc:self.log_notice.setText('日志读取失败：'+str(exc))

    def pairs(self):
        return validate_pairs([(s.currentData(),t.currentData()) for c,s,t in zip(self.pair_checks,self.leaders,self.followers)
                               if c.isChecked()],self.coordinator.roles)

    def start(self):
        try:
            self.error.clear();self.coordinator.start(self.pairs(),self.frequency.value(),self.mode.currentData(),self.rviz.isChecked())
        except Exception as exc:self.error.setText(str(exc))
        self.refresh()

    def release(self):
        self.close_pending=False;self.coordinator.release();self.refresh()

    def refresh(self):
        state=self.coordinator.snapshot();alive=self.coordinator.alive()
        busy=state['running'] or alive or bool(self.coordinator.worker and self.coordinator.worker.is_alive())
        for widget in self.configuration:widget.setEnabled(not busy)
        self.start_button.setEnabled(not busy);self.stop_button.setEnabled(busy);self.release_button.setEnabled(busy)
        self.status.setText(state['status'])
        self.status.setStyleSheet('color:#ffd0a0;background:#38291f;padding:13px;border-radius:6px' if state['phase']=='error' else '')
        errors=[]
        if state['error']:errors.append('首次故障：'+state['error'])
        if state.get('close_error'):errors.append('退出故障：'+state['close_error'])
        if state.get('log_error'):errors.append(state['log_error'])
        if errors:self.error.setText('\n'.join(errors))
        elif busy or state['phase']=='stopped':self.error.clear()
        for row,role in enumerate(('teaching_left','motion_left','teaching_right','motion_right')):
            arm=state['arms'].get(role,{});q=arm.get('hardware',{}).get('q')
            values=[self.coordinator.roles[role]['name'],arm.get('status','未选择 / 未连接'),
                    '  '.join(f'{x:+.3f}' for x in q[:6]) if q is not None else '—',
                    '已确认' if arm.get('release_confirmed') is True else '未确认' if arm else '—']
            for col,text in enumerate(values):
                item=QTableWidgetItem(text);item.setToolTip(text);self.table.setItem(row,col,item)
        self.metrics.setText(f"每组观测目标：{state.get('packets',0)}　目标观测平均：{state.get('actual_hz',0):.1f} Hz\n日志：{state.get('directory') or '尚未开始'}")
        self.refresh_logs(state)
        if self.close_pending:
            if state['close_failed'] or (not state['running'] and state['phase']=='error'):
                self.close_pending=False
            elif not busy and state['phase']=='stopped':
                self.close_pending=False;self.close()

    def closeEvent(self,event):
        state=self.coordinator.snapshot()
        if self.coordinator.alive() or state['running'] or (self.coordinator.worker and self.coordinator.worker.is_alive()):
            if not self.close_pending:self.close_pending=True;self.coordinator.stop()
            event.ignore()
        elif state.get('close_failed'):
            self.error.setText((state.get('close_error') or state['error'])+'；退出保护未完成，窗口保留');event.ignore()
        else:event.accept()


def main():
    parser=argparse.ArgumentParser(description='1 对 1 / 2 对 2 动力示教上位机')
    parser.add_argument('--check',action='store_true',help='离线检查模型、配对和参数，不连接硬件')
    args=parser.parse_args()
    if args.check:
        roles=load_roles();validate_pairs(DEFAULT_PAIRS,roles)
        for role in (r for pair in DEFAULT_PAIRS for r in pair):load_model(role,roles);port_parameters(role,roles)
        print('动力示教四臂模型、身份、默认配对及参数检查通过；没有连接硬件。');return
    app=QApplication(sys.argv[:1]);app.setFont(QFont('Noto Sans CJK SC',10))
    window=TeachingWindow();signal.signal(signal.SIGINT,lambda *_:window.close());signal.signal(signal.SIGTERM,lambda *_:window.close())
    def ui_exception(kind,value,tb):
        detail=''.join(traceback.format_exception(kind,value,tb))
        # The launcher also captures stderr before Qt or ROS can initialize.
        sys.__excepthook__(kind,value,tb)
        window.coordinator._event('ui_exception',level='ERROR',message=str(value),traceback=detail)
        window.log_notice.setText('界面异常已记录：'+str(value))
    sys.excepthook=ui_exception
    window.show();app.exec_()


if __name__=='__main__':main()
