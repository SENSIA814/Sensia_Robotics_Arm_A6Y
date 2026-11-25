"""Read Linux USB identities without opening a device or issuing USB/CAN IO."""
from pathlib import Path


def usb_snapshot(serials, usb_root=Path('/sys/bus/usb/devices'), tty_root=Path('/sys/class/tty')):
    devices = []
    tty_owners = []
    for pattern in ('ttyACM*', 'ttyUSB*'):
        for tty in tty_root.glob(pattern):
            try:
                target = tty.resolve(strict=True)
                # Only the nearest USB device owns this tty. Its upstream hubs
                # (including usb1 with serial 0000:00:14.0) are also ancestors,
                # but must never inherit the adapter's tty endpoints. Resolve
                # ownership before filtering serials, even for unknown devices.
                owner = next((parent for parent in target.parents
                              if (parent / 'idVendor').is_file()
                              and (parent / 'idProduct').is_file()), None)
                if owner is not None:
                    tty_owners.append((tty.name, owner))
            except OSError:
                continue
    for path in sorted(usb_root.glob('*')):
        try:
            serial = (path / 'serial').read_text().strip()
            if serial not in serials:
                continue
            device = {'serial': serial, 'usb_path': path.name}
            for field in ('idVendor', 'idProduct', 'product', 'manufacturer'):
                source = path / field
                device[field] = source.read_text().strip() if source.exists() else ''
            actual_path = path.resolve(strict=True)
            device['ttys'] = ['/dev/' + name for name, owner in tty_owners if actual_path == owner]
            device['mode'] = 'dfu' if (device['idVendor'], device['idProduct']) == ('0483', 'df11') else 'serial' if device['ttys'] else 'other'
            devices.append(device)
        except OSError:
            # A device may be unplugged while its sysfs attributes are read.
            continue
    return devices


def classify_connection(arm, devices):
    port = Path(arm['port'])
    exists = port.exists()
    resolved = str(port.resolve()) if exists else None
    matches = [d for d in devices if d['serial'] == arm['usb_serial']]
    result = {**arm, 'present': False, 'device': None, 'resolved_device': resolved,
              'usb_detected': bool(matches), 'usb_devices': matches,
              'connection_state': 'disconnected', 'connection_message': '未检测到 USB 适配器'}
    if len(matches) > 1:
        result.update(connection_state='duplicate_serial', connection_message='多个 USB 设备使用相同序列号，已禁止扫描')
    elif matches and matches[0]['mode'] == 'dfu':
        result.update(connection_state='dfu', connection_message='USB 已识别 · DFU 下载模式，无可用串口')
    elif matches and not exists:
        device = matches[0]
        text = ', '.join(device['ttys'])
        result.update(connection_state='unavailable', connection_message=
                      f'系统已识别 {text}，设备别名不可访问' if text else 'USB 已识别，但未生成串口')
    elif exists:
        # Linux tty numbers can be reused. Never trust a by-id symlink alone.
        is_usb_tty = Path(resolved).name.startswith(('ttyACM', 'ttyUSB'))
        owners = [d for d in devices if resolved in d['ttys']]
        if is_usb_tty and (len(owners) != 1 or owners[0]['serial'] != arm['usb_serial']):
            actual = ', '.join(d['serial'] for d in owners) or '无法核实'
            result.update(connection_state='identity_mismatch', connection_message=f'{resolved} 身份不符（实际：{actual}），已禁止扫描')
        else:
            # PTYs used by the integration harness have no USB parent.
            result.update(present=True, device=resolved, connection_state='ready', connection_message=f'已连接 {resolved}')
    return result


def connection_inventory(roles):
    devices = usb_snapshot({arm['usb_serial'] for arm in roles.values()})
    result = [classify_connection(arm, devices) for arm in roles.values()]
    aliases = {}
    for arm in result:
        if arm['resolved_device']:
            aliases.setdefault(arm['resolved_device'], []).append(arm)
    for device, linked in aliases.items():
        if len(linked) > 1:
            names = '、'.join(a['name'] for a in linked)
            for arm in linked:
                arm.update(present=False, device=None)
                arm['connection_message'] += f'；{names} 的别名共同指向 {device}'
                if arm['connection_state'] != 'dfu':
                    arm['connection_state'] = 'alias_conflict'
    return result


def verify_usb_identity(arm):
    """Check a real USB serial endpoint immediately before SLCAN initialization."""
    port = Path(arm['port'])
    actual = port.resolve()
    # Preserve the existing PTY test path; hardware tty endpoints must match.
    if actual.name.startswith(('ttyACM', 'ttyUSB')) or str(port).startswith('/dev/serial/by-id/'):
        # Include all serials so a stale alias can be attributed to its owner.
        serials = {arm['usb_serial']}
        for candidate in Path('/sys/bus/usb/devices').glob('*/serial'):
            try:
                serials.add(candidate.read_text().strip())
            except OSError:
                pass
        state = classify_connection(arm, usb_snapshot(serials))
        if not state['present']:
            raise RuntimeError(state['connection_message'])
        return state
    return None
