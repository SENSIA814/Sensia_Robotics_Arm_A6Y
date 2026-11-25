"""Coordinate global diagnostic operations and independent gravity sessions."""
from contextlib import ExitStack, contextmanager
import fcntl
import hashlib
import os
from pathlib import Path


LOCK_PATH = Path('/tmp') / f'a6y-arm-diagnostics-{os.getuid()}.lock'


@contextmanager
def _file_lock(path, mode, message):
    # Never unlink a flock file: another process may already hold its inode.
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'r+') as stream:
        try:
            fcntl.flock(stream.fileno(), mode | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(message) from None
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


@contextmanager
def exclusive_operation():
    """Keep existing scans, calibration and motion tools globally exclusive."""
    with _file_lock(LOCK_PATH, fcntl.LOCK_EX,
                    '另一个窗口或命令正在连接机械臂、扫描或标定，请先停止；这些功能不能同时运行'):
        yield


@contextmanager
def arm_operation(arm):
    """Allow gravity on distinct arms while excluding every global operation.

    Lock both the stable USB identity and resolved endpoint. The second lock
    also rejects misconfigured roles/aliases pointing at the same serial port.
    Lock files are never removed, including after unplugging a USB adapter.
    This only opens lock files; USB identity validation stays with the caller.
    """
    serial = str(arm.get('usb_serial', '')).strip()
    port = str(arm.get('port', '')).strip()
    if not serial or not port:
        raise ValueError('机械臂缺少 USB 序列号或串口配置，不能取得独立控制权')
    keys = ('usb:' + serial, 'port:' + str(Path(port).resolve()))
    paths = sorted(LOCK_PATH.with_name(LOCK_PATH.name + '.' +
                   hashlib.sha256(key.encode('utf-8')).hexdigest() + '.arm') for key in keys)
    name = arm.get('name') or serial
    with ExitStack() as locks:
        locks.enter_context(_file_lock(LOCK_PATH, fcntl.LOCK_SH,
                            '正在扫描、标定或执行其他控制任务，不能同时运行重力补偿；请先停止该任务'))
        for path in paths:
            locks.enter_context(_file_lock(path, fcntl.LOCK_EX,
                                f'{name} 的 USB 设备或串口已由其他会话占用，请先断开该机械臂'))
        yield
