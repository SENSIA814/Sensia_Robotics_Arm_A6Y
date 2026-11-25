#!/usr/bin/env python3
"""Program one isolated Damiao motor's ESC_ID/MST_ID through CANable2/SLCAN."""
from __future__ import annotations

import argparse
import fcntl
import math
import struct
import sys
import time
from contextlib import ExitStack

from damiao_bus_scan import find_driver_root


ESC_ID = 8
MST_ID = 7
CTRL_MODE = 10
PARAM_CAN_ID = 0x7FF


class IdProgrammer:
    """Use fresh, strictly matched replies, independent of the driver's ID cache.

    Only use with one physically isolated motor. A scan cannot distinguish two
    motors with identical IDs, nor discover silent devices or every CAN device.
    """

    def __init__(self, control, timeout=0.25, report=print, read_attempts=3, trace=False):
        self.control = control
        self.timeout = timeout
        self.report = report
        self.read_attempts = read_attempts
        self.trace = trace
        self.stage = 'preflight'
        self.write_attempted = False
        self.save_attempted = False

    def clear_rx(self):
        for _ in range(256):
            msg = self.control.bus.recv(timeout=0.0)
            if msg is None:
                return
            self.trace_rx('DRAIN RX', msg)
        raise RuntimeError('CAN receive queue did not drain; stop other bus traffic')

    def trace_rx(self, direction, msg):
        if self.trace:
            flags = [name for name in ('is_extended_id', 'is_remote_frame', 'is_error_frame')
                     if getattr(msg, name, False)]
            self.report(f'{direction} CAN=0x{msg.arbitration_id:03X} DATA={bytes(msg.data).hex().upper()} '
                        f'DLC={getattr(msg, "dlc", len(msg.data))} FLAGS={",".join(flags) or "std"}')

    def send_frame(self, can_id, data):
        if self.trace:
            self.report(f'TX CAN=0x{can_id:03X} DATA={data.hex().upper()}')
        self.control.send_std(can_id, data)

    @staticmethod
    def frame_data(msg, lengths=(8,)):
        if (msg is None or msg.is_extended_id
                or getattr(msg, 'is_remote_frame', False)
                or getattr(msg, 'is_error_frame', False)):
            return None
        data = bytes(msg.data)
        if len(data) not in lengths or getattr(msg, 'dlc', len(data)) != len(data):
            return None
        if not 0 <= msg.arbitration_id < PARAM_CAN_ID:
            return None
        return data

    def receive(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            msg = self.control.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is not None:
                self.trace_rx('RX', msg)
                yield msg

    def send_param(self, slave, opcode, rid, value=0):
        self.send_frame(PARAM_CAN_ID, struct.pack('<HBBI', slave, opcode, rid, value))

    def read_uint(self, slave, rid, master=None, *, required=True):
        replies = set()
        # Only repeat reads with no matching reply, never writes or flash saves.
        # An empty address during the isolation scan needs only one query.
        attempts = self.read_attempts if required else 1
        for attempt in range(1, attempts + 1):
            self.clear_rx()
            self.send_param(slave, 0x33, rid)
            for msg in self.receive():
                data = self.frame_data(msg)
                if data is not None and data[:4] == struct.pack('<HBB', slave, 0x33, rid):
                    replies.add((msg.arbitration_id, struct.unpack('<I', data[4:])[0]))
            if replies:
                break
            if attempt < attempts:
                self.report(f'RETRY read ESC_ID=0x{slave:02X} RID={rid}: no reply; '
                            f'attempt {attempt + 1}/{attempts}')
        if len(replies) > 1:
            raise RuntimeError(f'Conflicting replies at ESC_ID=0x{slave:02X}, RID={rid}: {sorted(replies)}')
        if not replies:
            if not required:
                return None
            raise RuntimeError(f'No fresh parameter reply at ESC_ID=0x{slave:02X}, RID={rid} '
                               f'after {attempts} attempts ({self.timeout:g}s each); '
                               'use --diagnose --trace to inspect read-only traffic')
        sender, value = replies.pop()
        if master is not None and sender != master:
            raise RuntimeError(f'Unexpected feedback ID 0x{sender:02X}; expected 0x{master:02X}')
        return sender, value

    def read_pair(self, slave, master=None):
        sender, value = self.read_uint(slave, ESC_ID, master)
        if value != slave:
            raise RuntimeError(f'ESC_ID readback mismatch: expected {slave}, got {value}')
        _, feedback = self.read_uint(slave, MST_ID, sender)
        if feedback != sender:
            raise RuntimeError(f'MST_ID register 0x{feedback:X} differs from reply CAN ID 0x{sender:X}')
        self.report(f'READ ESC_ID=0x{slave:02X} MST_ID=0x{feedback:02X}')
        return feedback

    def check_isolated(self, current):
        # This supplements physical isolation; it is not proof of an empty bus.
        self.stage = 'check other motor IDs'
        self.report(f'CHECK other motor IDs 1..15 (reply windows total {14 * self.timeout:g}s); read-only queries.')
        for candidate in range(1, 16):
            if candidate == current:
                continue
            if self.read_uint(candidate, ESC_ID, required=False) is not None:
                raise RuntimeError(f'Another motor replied at 0x{candidate:02X}; connect only the target motor')

    def read_state(self, slave, master):
        for attempt in range(1, self.read_attempts + 1):
            self.clear_rx()
            self.send_param(slave, 0xCC, 0)
            states = set()
            for msg in self.receive():
                data = self.frame_data(msg)
                if (data is not None and msg.arbitration_id == master
                        and data[0] & 0x0F == slave):
                    # Do not mistake a delayed parameter ACK for disabled state.
                    if data[1] == 0 and data[2] in (0x33, 0x55, 0xAA):
                        continue
                    states.add(data[0] >> 4)
            if len(states) > 1:
                raise RuntimeError(f'Conflicting status replies: {sorted(states)}')
            if states:
                state = states.pop()
                label = {0: 'disabled', 1: 'enabled'}.get(state, 'fault/unknown')
                self.report(f'STATE={state} ({label})')
                return state
            if attempt < self.read_attempts:
                self.report(f'RETRY status ESC_ID=0x{slave:02X}: no reply; '
                            f'attempt {attempt + 1}/{self.read_attempts}')
        raise RuntimeError(f'No fresh status reply at ESC_ID=0x{slave:02X}, MST_ID=0x{master:02X} '
                           f'after {self.read_attempts} attempts; disabled state is unknown')

    def ensure_disabled(self, slave, master):
        self.stage = 'check disabled state'
        state = self.read_state(slave, master)
        if state == 0:
            return  # Already disabled: no need to know or change its control mode.
        if state != 1:
            raise RuntimeError(f'Disabled state not confirmed (state={state}); no parameter write permitted')
        self.stage = 'read control mode before disable'
        _, mode = self.read_uint(slave, CTRL_MODE, master)
        if mode not in range(1, 8):
            raise RuntimeError(f'Unsupported CTRL_MODE={mode}; cannot select disable command ID')
        self.report(f'CTRL_MODE={mode}')
        self.stage = 'disable'
        # Enabled motor: the mode is required to select the command CAN ID.
        self.clear_rx()
        self.send_frame(slave + (mode - 1) * 0x100, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.02)
        if self.read_state(slave, master) != 0:
            raise RuntimeError('Disabled state not confirmed after disable; no parameter write permitted')

    def diagnose(self, slave, expected_master):
        """Collect both mode and state even if one of their reads times out."""
        self.stage = 'diagnose IDs'
        master = self.read_pair(slave, expected_master)
        complete = True
        self.stage = 'diagnose control mode'
        try:
            _, mode = self.read_uint(slave, CTRL_MODE, master)
            self.report(f'CTRL_MODE={mode}')
        except RuntimeError as exc:
            self.report(f'DIAG CTRL_MODE: {exc}')
            complete = False
        self.stage = 'diagnose status'
        try:
            self.read_state(slave, master)
        except RuntimeError as exc:
            self.report(f'DIAG STATUS: {exc}')
            complete = False
        self.report(f'DIAGNOSE {"complete" if complete else "incomplete"}: no disable/write/save sent.')
        return complete

    def change(self, slave, master, rid, value):
        self.stage = f'write {"MST_ID" if rid == MST_ID else "ESC_ID"}'
        self.report(f'WRITE ESC_ID=0x{slave:02X} RID={rid} -> 0x{value:02X}')
        self.clear_rx()
        self.write_attempted = True  # Set before send: a transport error can follow delivery.
        self.send_param(slave, 0x55, rid, value)
        # The write ACK may use either address. Never use it as readback proof.
        time.sleep(0.05)
        new_slave = value if rid == ESC_ID else slave
        new_master = value if rid == MST_ID else master
        self.read_pair(new_slave, new_master)
        return new_slave, new_master

    def save(self, slave, master):
        self.stage = 'save parameters'
        self.clear_rx()
        self.save_attempted = True
        # The manual specifies AA 01; the general driver historically defaults
        # to AA 00. Send the documented code explicitly and require its ACK.
        self.send_param(slave, 0xAA, 0x01)
        acknowledged = False
        for msg in self.receive():
            data = self.frame_data(msg, lengths=(4, 8))
            if (data is not None and msg.arbitration_id == master
                    and data[:4] == struct.pack('<HBB', slave, 0xAA, 0x01)):
                acknowledged = True
        time.sleep(0.1)  # Keep power/connection available beyond the flash write time.
        self.read_pair(slave, master)
        if not acknowledged:
            raise RuntimeError('SAVE_UNCONFIRMED: IDs read back, but no save ACK; persistence is unknown. '
                               'Do not blindly repeat flash writes; power-cycle and use --verify')

    def program(self, current, expected_master, new_slave, new_master):
        master = self.read_pair(current, expected_master)
        self.check_isolated(current)
        self.ensure_disabled(current, master)
        if master != new_master:
            current, master = self.change(current, master, MST_ID, new_master)
        if current != new_slave:
            current, master = self.change(current, master, ESC_ID, new_slave)
        self.ensure_disabled(current, master)
        self.save(current, master)
        self.stage = 'complete'
        self.report(f'SAVED ESC_ID=0x{current:02X} MST_ID=0x{master:02X}; save ACK and readback received. '
                    'Power-cycle, then --verify to check persistence.')


def integer(text):
    return int(text, 0)


def positive_timeout(text):
    value = float(text)
    if not math.isfinite(value) or not 0.05 <= value <= 5.0:
        raise argparse.ArgumentTypeError('timeout must be between 0.05 and 5 seconds')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog=
        'Default: offline preview. --write disables and saves parameters on one physically isolated motor. '
        'Support the arm and stop all other serial/CAN users first. YAML files are not modified.')
    parser.add_argument('--channel', '--port', default='/dev/ttyACM0')
    parser.add_argument('--can-bitrate', type=int, default=1000000)
    parser.add_argument('--tty-baudrate', type=int, default=115200)
    parser.add_argument('--current-id', type=integer, required=True, help='Current ESC_ID, 1..15 (decimal or 0x hex).')
    parser.add_argument('--current-master-id', type=integer, help='Expected current MST_ID; otherwise read automatically.')
    parser.add_argument('--new-id', type=integer, help='New ESC_ID, 1..15, required for preview/write.')
    parser.add_argument('--new-master-id', type=integer, help='New MST_ID, 0x10..0x7FE; default: new ID + 0x10.')
    parser.add_argument('--timeout', type=positive_timeout, default=0.25, help='Reply window in seconds (default: 0.25).')
    parser.add_argument('--read-attempts', type=int, default=3, help='Read attempts on missing replies, 1..10 (default: 3).')
    parser.add_argument('--trace', action='store_true', help='Print raw CAN TX/RX frames for diagnosis.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run', action='store_true', help='Offline preview; default, no serial access.')
    mode.add_argument('--write', action='store_true', help='Ensure disabled, write both IDs, read back and save to flash.')
    mode.add_argument('--verify', action='store_true', help='Read current IDs only; use after a power cycle.')
    mode.add_argument('--diagnose', action='store_true', help='Read IDs, control mode and state; never disable/write/save.')
    args = parser.parse_args(argv)
    if not 1 <= args.current_id <= 15:
        parser.error('--current-id must be 1..15 (this project uses 4-bit feedback motor IDs)')
    if args.current_master_id is not None and not 0 <= args.current_master_id <= 0x7FE:
        parser.error('--current-master-id must be 0..0x7FE')
    if args.can_bitrate not in (10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000):
        parser.error('--can-bitrate must be a supported SLCAN bitrate')
    if args.tty_baudrate <= 0:
        parser.error('--tty-baudrate must be positive')
    if not 1 <= args.read_attempts <= 10:
        parser.error('--read-attempts must be 1..10')
    if args.verify or args.diagnose:
        if args.new_id is not None or args.new_master_id is not None:
            parser.error('--verify/--diagnose take expected IDs in --current-id / --current-master-id, not --new-id')
    else:
        if args.new_id is None or not 1 <= args.new_id <= 15:
            parser.error('--new-id must be 1..15')
        if args.new_master_id is None:
            args.new_master_id = args.new_id + 0x10
        if not 0x10 <= args.new_master_id <= 0x7FE:
            parser.error('--new-master-id must be 0x10..0x7FE, separate from motor IDs and reserved 0x7FF')
        print(f'PLAN {args.channel}: ESC_ID 0x{args.current_id:02X} -> 0x{args.new_id:02X}, '
              f'MST_ID -> 0x{args.new_master_id:02X}; ensure disabled, readback, save (AA 01).', flush=True)
        if not args.write:
            print('DRY RUN: no serial access. Add --write to execute on one isolated motor.')
            return 0

    programmer = None
    try:
        sys.path.insert(0, str(find_driver_root()))
        from dm_canable2 import MotorControl
        with ExitStack() as stack:
            # Match the C++ driver and feedback watch lock before SLCAN setup.
            device = stack.enter_context(open(args.channel, 'rb', buffering=0))
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            control = MotorControl(channel=args.channel, can_bitrate=args.can_bitrate,
                                   tty_baudrate=args.tty_baudrate, auto_open=False)
            stack.callback(control.close)
            programmer = IdProgrammer(control, args.timeout, report=lambda line: print(line, flush=True),
                                      read_attempts=args.read_attempts, trace=args.trace)
            programmer.stage = 'open serial device'
            print(f'OPEN {args.channel}: initializing SLCAN...', flush=True)
            control.open()
            print('CONNECTED', flush=True)
            programmer.stage = 'read current IDs'
            if args.diagnose:
                return 0 if programmer.diagnose(args.current_id, args.current_master_id) else 2
            elif args.verify:
                programmer.read_pair(args.current_id, args.current_master_id)
                print('VERIFIED: current IDs read back; no disable/write/save sent.')
            else:
                programmer.program(args.current_id, args.current_master_id, args.new_id, args.new_master_id)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        stage = programmer.stage if programmer is not None else 'open/lock serial device'
        print(f'ERROR [{stage}]: {str(exc) or "interrupted"}', file=sys.stderr)
        if programmer is not None and (programmer.write_attempted or programmer.save_attempted):
            print('IDs may be partially changed; no automatic rollback or write/save retry was sent. '
                  f'Keep this motor isolated; check ESC_ID 0x{args.current_id:02X} and 0x{args.new_id:02X} '
                  'with --verify (omit --current-master-id to discover feedback ID). '
                  'RAM readback alone does not prove flash persistence.', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 2


if __name__ == '__main__':
    raise SystemExit(main())
