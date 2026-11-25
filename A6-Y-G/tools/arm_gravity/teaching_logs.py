"""Bounded desktop log reader and self-contained teaching diagnostic bundles."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import threading
import time
import zipfile


def write_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
    os.replace(temp, path)


def event_text(event):
    stamp = datetime.fromtimestamp(event.get('time', time.time())).astimezone().isoformat(timespec='milliseconds')
    detail = {k:v for k,v in event.items() if k not in ('time','kind','level')}
    return f"{stamp} [{event.get('level','INFO')}] {event.get('kind','event')} " + json.dumps(detail, ensure_ascii=False)


class SessionLog:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.lock = threading.Lock()

    def event(self, kind, level='INFO', **data):
        entry = dict(time=time.time(), level=level, kind=kind, **data)
        with self.lock:
            with (self.folder/'events.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(entry, ensure_ascii=False, default=str)+'\n')
            with (self.folder/'session.log').open('a', encoding='utf-8') as stream:
                stream.write(event_text(entry)+'\n')

    def snapshot(self, state):
        with self.lock:
            write_json(self.folder/'result.json', state)


def software_manifest(root):
    files = list((root/'tools/arm_gravity').glob('*.py'))
    files += [root/'tools/arm_gravity/motor_limits.json']
    files += [root/'tools/arm_moveit/native_stack.py']
    files += list((root/'src/a6_y_controllers/src').glob('*.cpp'))
    files += list((root/'src/a6_y_ros2_control/src').glob('*.cpp'))
    files += list((root/'src/a6_y_controllers/include').rglob('*.hpp'))
    files += list((root/'src/a6_y_ros2_control/include').rglob('*.hpp'))
    files += list((root/'src/a6_y_teleop/a6_y_teleop').glob('*.py'))
    files += list((root/'src/a6_y_teleop/config').glob('*.yaml'))
    files += [root/'src/a6_y_ros2_control/tools/move_to_group_state.py']
    files += list((root/'src/a6_y_teleop_msgs/msg').glob('*.msg'))
    files += list((root/'src/a6_y_teleop_msgs/srv').glob('*.srv'))
    return dict(python=platform.python_version(), platform=platform.platform(),
                ros={k:os.environ.get(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION')},
                sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})


def report_arms(folder, state=None):
    folder = Path(folder)
    if state is None:
        state = json.loads((folder/'result.json').read_text(encoding='utf-8')) if (folder/'result.json').is_file() else {}
    result = {}
    # New bundles are relocatable; older reports keep their original arm paths.
    for role, arm in state.get('arms',{}).items():
        local = folder/'arms'/role
        path = local if local.is_dir() else Path(arm.get('directory') or local)
        if path.is_dir(): result[role] = path
    return result


def summary_text(state):
    lines = ['A6Y 动力示教诊断', f"模式：{'离线' if state.get('mock') else '真机'} / {state.get('frequency','—')} Hz",
             f"阶段：{state.get('phase')} / {state.get('status')}", f"首次故障：{state.get('error') or '无'}",
             f"退出故障：{state.get('close_error') or '无'}", f"日志：{state.get('directory') or '尚未开始'}"]
    if state.get('log_error'):lines.append(state['log_error'])
    rates=state.get('configured_rates')
    if rates:
        lines.append(f"配置：底层 {rates['control_hz']} Hz / 每轴 CAN 指令 {rates['per_axis_command_hz']} Hz / "
                     f"目标 {rates['target_hz']} Hz / CAN {rates['can_bitrate']/1e6:g} Mbps")
    for role,arm in state.get('arms',{}).items():
        lines.append(f"{role} [{arm.get('backend','—')}] {arm.get('status','')}；失能确认={arm.get('release_confirmed')}；"
                     f"{arm.get('error') or arm.get('notice','')}")
    return '\n'.join(lines)


def export_report(folder, destination, state=None):
    """Copy only this report's files, with a fixed per-file size while running.

    No symlinks, unrelated arm sessions, or arbitrary paths in an imported JSON
    report are followed. A manifest records byte counts and snapshot timing.
    """
    folder, destination = Path(folder).resolve(), Path(destination).resolve()
    if not folder.is_dir(): raise ValueError('日志目录不存在')
    if state is None:
        state = json.loads((folder/'result.json').read_text(encoding='utf-8'))
    temp = destination.with_name(destination.name+'.partial')
    allowed_arms = (folder.parent/'arms').resolve()
    files = {}
    def collect(base, prefix):
        for path in base.rglob('*'):
            if not path.is_file() or path.is_symlink() or path in (destination,temp): continue
            if path.suffix in ('.tmp','.zip','.partial'): continue
            resolved = path.resolve()
            if not resolved.is_relative_to(base.resolve()): continue
            name = str(Path(prefix)/path.relative_to(base))
            files[name] = (path, path.stat().st_size)
    collect(folder, '')
    metadata=json.loads((folder/'session.json').read_text()) if (folder/'session.json').is_file() else {}
    if metadata.get('application_log'):
        app_log=Path(metadata['application_log'])
        if (app_log.is_file() and not app_log.is_symlink() and
                app_log.resolve().is_relative_to((folder.parent.parent/'arm_teaching_logs').resolve())):
            files['application.log']=(app_log,app_log.stat().st_size)
    arms=report_arms(folder,state)
    for role,path in arms.items():
        if role not in ('teaching_left','teaching_right','motion_left','motion_right'): continue
        if not (path.resolve().is_relative_to(allowed_arms) or path.resolve().is_relative_to(folder/'arms')):
            raise ValueError('机械臂日志路径不在本报告的日志区域内：'+role)
        collect(path, 'arms/'+role)
    manifest = dict(created=time.time(), live=bool(state.get('running') or any(a.get('running') for a in state.get('arms',{}).values())),
                    source=str(folder), files={name:size for name,(_,size) in files.items()},
                    missing_arm_logs=[role for role in state.get('arms',{}) if role not in arms],
                    note='各文件按打包开始时的长度复制；运行中导出不停止控制。')
    try:
        with zipfile.ZipFile(temp,'w',compression=zipfile.ZIP_DEFLATED) as bundle:
            for name,(path,size) in files.items():
                if name in ('result.json','summary.txt','bundle.json'): continue
                with path.open('rb') as source, bundle.open(name,'w') as target:
                    remaining=size
                    while remaining:
                        chunk=source.read(min(remaining,1024*1024))
                        if not chunk: break
                        target.write(chunk);remaining-=len(chunk)
            bundle.writestr('result.json',json.dumps(state,ensure_ascii=False,indent=2,default=str))
            bundle.writestr('summary.txt',summary_text(state))
            bundle.writestr('bundle.json',json.dumps(manifest,ensure_ascii=False,indent=2))
        os.replace(temp,destination)
    finally:
        temp.unlink(missing_ok=True)
    return destination


class LogTail:
    """Incremental UTF-8 line reader; never read whole growing telemetry files."""
    def __init__(self): self.cursors={}

    def read(self, folder, role='all', state=None, detailed=True):
        folder=Path(folder); sources=[]
        if role in ('all','coordinator'):
            sources.append(('整组',folder/'events.jsonl',True))
        for name,path in report_arms(folder,state).items():
            if role in ('all',name):
                sources.append((name,path/'events.jsonl',True))
                sources.append((name+'/driver',path/'controller_manager.log',False))
                sources.append((name+'/planner',path/'move_group.log',False))
                sources.append((name+'/native_home',path/'native_home.log',False))
                sources.append((name+'/mapper',folder/(name+'_mapper.log'),False))
        lines=[]
        for label,path,structured in sources:
            if not path.is_file(): continue
            size=path.stat().st_size
            cursor=self.cursors.get(path,max(0,size-32768))
            if cursor>size: cursor=0
            with path.open('rb') as source:
                source.seek(cursor);data=source.read(65536)
            end=data.rfind(b'\n')+1
            self.cursors[path]=cursor+end
            for raw in data[:end].decode('utf-8',errors='replace').splitlines():
                if not structured and not detailed and not any(word in raw.upper() for word in ('ERROR','WARN','FATAL','TRACEBACK')):
                    continue
                if structured:
                    try: raw=event_text(json.loads(raw))
                    except (ValueError,TypeError): pass
                lines.append(f'[{label}] {raw}')
        return lines
