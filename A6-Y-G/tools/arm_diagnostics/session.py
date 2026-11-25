"""Mutually exclusive background scanning and native zero calibration."""
from copy import deepcopy
import secrets
import threading

from scanner import save_report, scan_selected, validate_options, validate_selection
from zero_calibration import calibrate_zero


class ScanSession:
    def __init__(self, roles, report_dir):
        self.roles = roles
        self.report_dir = report_dir
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.worker = None
        self.state = {"running": False, "operation": "scan", "progress": 0, "total": 0,
                      "message": "请选择本次要扫描的机械臂", "report": None,
                      "report_path": None, "error": None, "selected": [], "motors": {}}

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def start(self, selected, passes):
        selected = validate_selection(selected, self.roles)
        validate_options(passes, .06)
        self._start(selected, passes, "scan")

    def start_zero(self, role):
        if not isinstance(role, str) or role not in self.roles:
            raise ValueError("写零位必须明确选择一条机械臂")
        self._start([role], 1, "zero")

    def _start(self, selected, passes, operation):
        with self.lock:
            if self.state["running"]:
                raise RuntimeError("已有扫描或标定正在运行，请等待完成或停止")
            self.cancel = threading.Event()
            self.state = {"running": True, "operation": operation, "progress": 0, "total": len(selected) * passes * 7,
                          "message": "正在准备串口", "selected": selected, "passes": passes,
                          "report": None, "report_path": None, "error": None, "motors": {}}
            self.worker = threading.Thread(target=self._run, args=(selected, passes, operation), daemon=True)
            self.worker.start()

    def _emit(self, event):
        with self.lock:
            if event["event"] == "arm_start":
                self.state["message"] = f"正在扫描 {event['name']}"
            elif event["event"] == "progress":
                self.state["progress"] += 1
                self.state["motors"][f"{event['role']}:{event['motor_id']}"] = deepcopy(event["motor"])
                self.state["message"] = f"{self.roles[event['role']]['name']} · 第 {event['round']} 轮 · ID {event['motor_id']}"
            elif event["event"] == "zero_progress":
                self.state["motors"][f"{event['role']}:{event['motor']['id']}"] = deepcopy(event["motor"])
                self.state["progress"] = sum(m.get("verified", False) for m in self.state["motors"].values())
                self.state["message"] = event["message"]
            elif event["event"] == "arm_done":
                arm = event["arm"]
                for motor in arm["motors"]:
                    self.state["motors"][f"{arm['role']}:{motor['id']}"] = deepcopy(motor)

    def _run(self, selected, passes, operation):
        report = None
        path = None
        error = None
        try:
            if operation == "zero":
                report = calibrate_zero(selected[0], self.roles, cancel=self.cancel, emit=self._emit)
            else:
                report = scan_selected(selected, self.roles, passes=passes, cancel=self.cancel, emit=self._emit)
            path = str(save_report(report, self.report_dir))
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        with self.lock:
            message = "扫描完成"
            if error:
                message = "扫描或报告保存失败"
            elif report["cancelled"]:
                message = "扫描已停止，部分结果已保存"
            elif report["ok"]:
                message = "本轮通信检查通过"
            else:
                message = "扫描结束，存在需要检查的项目"
            if operation == "zero":
                if error:
                    message = "标定或报告保存失败，请检查逐电机结果"
                elif report["ok"]:
                    message = "零位标定完成 · 7 / 7 台电机写入并回读通过"
                else:
                    arm = report["arms"][0]
                    message = ("标定已停止" if report["cancelled"] else "标定未全部完成")
                    message += f" · 已验证 {len(arm['verified_ids'])} / 7；未确认 {arm['uncertain_ids']}；未写入 {arm['not_written_ids']}"
            self.state.update(running=False, report=report, report_path=path, error=error, message=message)
