#!/usr/bin/env python3
"""Concurrent native gravity controllers on PTYs; never opens physical ports."""
import argparse
from copy import deepcopy
import json
import multiprocessing
import os
import struct
from pathlib import Path
import threading
import time

import numpy as np

from gravity_config import ROLE_ORDER, ROOT, load_roles
from gravity_session import GravitySession
from verify_gravity import MotorEmulator, wait


def _emulator_process(role, connection, position_mode=False, native_port=False):
    """Model a USB adapter independently of the UI's Python interpreter/GIL."""
    emulator = None
    try:
        emulator = MotorEmulator(role,position_mode=position_mode,native_port=native_port)
        cursor = 0
        audit = dict(frames=0, enables=0, disables=0, nonqueries=0,
                     gripper_commands=0, zero_commands=0, position_commands={})

        def snapshot():
            nonlocal cursor
            frames = emulator.frames[cursor:]
            cursor += len(frames)
            for cid, data in frames:
                audit['enables'] += data == b'\xff' * 7 + b'\xfc'
                audit['disables'] += data == b'\xff' * 7 + b'\xfd'
                audit['nonqueries'] += not (cid == 0x7ff and data[2] == 0xcc)
                audit['gripper_commands'] += cid in (7,0x107)
                audit['zero_commands'] += data == b'\xff' * 7 + b'\xfe'
                if 0x101 <= cid <= 0x106:
                    i = cid - 0x100
                    entry = emulator.model['joints']['joint'+str(i)]
                    q = entry['sign'] * struct.unpack('<ff', data)[0] * entry['position_scale'] + entry['offset']
                    values = audit['position_commands'].setdefault(i, dict(first=q, min=q, max=q, count=0))
                    values.update(min=min(values['min'], q), max=max(values['max'], q), count=values['count']+1)
            audit['frames'] = cursor
            return dict(audit, port=emulator.port, enabled=sorted(emulator.enabled),
                        commands=len(emulator.commands),
                        last_torque=emulator.last_torque.copy(),
                        last_position=emulator.last_position.copy(),mode_writes=list(emulator.mode_writes),
                        max_speed=emulator.max_speed.copy(),q=emulator.q.tolist(),
                        failure=repr(emulator.failure) if emulator.failure else None)

        connection.send(snapshot())
        while True:
            command = connection.recv()
            if isinstance(command,tuple) and command[0]=='set_q':
                q=np.asarray(command[1],dtype=float)
                assert q.shape==(7,) and np.isfinite(q).all()
                emulator.q=q.copy()
            elif command == 'freeze_motion':
                emulator.freeze_motion = True
            elif command == 'reject_mode':
                emulator.reject_mode = True
            elif command == 'restore_motion':
                emulator.freeze_motion = False
            elif command == 'drop_feedback':
                emulator.drop_feedback = True
            elif command == 'close':
                state = snapshot()
                emulator.close()
                emulator = None
                connection.send(state)
                break
            elif command != 'snapshot':
                raise ValueError(command)
            connection.send(snapshot())
    except EOFError:
        pass
    except BaseException as exc:
        try:
            connection.send(dict(failure=repr(exc)))
        except (BrokenPipeError, EOFError):
            pass
    finally:
        if emulator:
            emulator.close()
        connection.close()


class ProcessMotorEmulator:
    def __init__(self, role, position_mode=False, native_port=False):
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.lock = threading.Lock()
        self.worker = context.Process(target=_emulator_process, args=(role, child,position_mode,native_port), daemon=True)
        self.worker.start()
        child.close()
        assert self.connection.poll(15), 'PTY emulator did not start'
        state = self.connection.recv()
        assert not state.get('failure'), state
        self.port = state['port']
        assert self.port.startswith('/dev/pts/')

    def snapshot(self, command='snapshot'):
        with self.lock:
            self.connection.send(command)
            assert self.connection.poll(5), 'PTY emulator did not answer'
            state = self.connection.recv()
            assert not state.get('failure'), state
            return state

    def close(self):
        try:
            if self.worker.is_alive():
                self.snapshot('close')
        finally:
            self.worker.join(5)
            if self.worker.is_alive():
                self.worker.terminate()
                self.worker.join(3)
            self.connection.close()
        assert self.worker.exitcode == 0, 'PTY emulator failed to stop cleanly'


class ContinuityMonitor:
    """Continuously check peers while another session changes or fails."""
    def __init__(self, peers):
        self.peers = peers
        self.baselines = {role: (session.snapshot()['applied_parameters'],
                                np.array(session.snapshot()['commanded']),
                                emulator.snapshot())
                          for role, (session, emulator) in peers.items()}
        self.finished = threading.Event()
        self.failure = None
        self.samples = 0
        self.observer_resyncs = 0
        self.max_observer_resync_seconds = 0.
        self.worker = threading.Thread(target=self.run, daemon=True)

    def run(self):
        last = {role: (em.snapshot()['commands'], time.monotonic()) for role, (_, em) in self.peers.items()}
        try:
            while not self.finished.is_set():
                for role, (session, emulator) in self.peers.items():
                    state = session.snapshot()
                    parameters, command, initial_motor = self.baselines[role]
                    assert state['active'] and state['ready'] and not state['error'], (role, state)
                    if not session._fresh(enabled=True):
                        # The observer may resume before the session executor
                        # after a Python-wide ROS teardown pause. Permit one
                        # bounded callback drain, while active/error and every
                        # motor FC/FD frame remain strict invariants below.
                        started = time.monotonic()
                        wait(session, lambda current: current['active'] and
                             session._fresh(enabled=True), timeout=.1)
                        self.observer_resyncs += 1
                        self.max_observer_resync_seconds = max(
                            self.max_observer_resync_seconds, time.monotonic()-started)
                    assert state['applied_parameters'] == parameters, role + ': peer parameters changed'
                    np.testing.assert_allclose(state['commanded'], command, atol=1e-8, rtol=0)
                    motor = emulator.snapshot()
                    assert (motor['enables'], motor['disables']) == (initial_motor['enables'], initial_motor['disables']), role + ': unexpected peer enable/disable'
                    assert motor['enabled'] == list(range(1, 7)), role + ': peer motor enable changed'
                    count, stamp = last[role]
                    if motor['commands'] > count:
                        last[role] = (motor['commands'], time.monotonic())
                    else:
                        assert time.monotonic() - stamp < .5, role + ': peer MIT commands stopped'
                self.samples += 1
                self.finished.wait(.02)
        except BaseException as exc:
            self.failure = exc

    def check(self):
        if self.failure:
            raise AssertionError('Unrelated arm continuity failed') from self.failure

    def close(self):
        self.finished.set()
        self.worker.join(3)
        self.check()


def cleanup(sessions):
    for session in sessions:
        session.release()
    for session in sessions:
        if session.worker and session.worker.is_alive():
            try:
                wait(session, lambda state: not state['running'] or
                     (state['release_confirmed'] and not state['busy']), timeout=10, allow_error=True)
            except AssertionError:
                # This is a test failure cleanup, never a planned movement.
                session.stop_event.set()
        session.stop()
    for session in sessions:
        if session.worker:
            session.worker.join(25)
            assert not session.worker.is_alive(), 'test worker failed to stop'


def verify(selected):
    assert len(selected) >= 2 and len(set(selected)) == len(selected)
    emulators, sessions, histories = {}, {}, []
    monitor = None
    stages = []
    started = time.monotonic()
    result = dict(result='FAIL', physical_devices=0, roles=selected, stages=stages,
                  virtual_hardware='one independent process per PTY adapter',
                  cyclonedds_config=os.environ.get('CYCLONEDDS_URI'))
    try:
        for role in selected:
            emulators[role] = ProcessMotorEmulator(role)
        roles = load_roles()
        for role, emulator in emulators.items():
            assert emulator.port.startswith('/dev/pts/')
            roles[role]['port'] = emulator.port
        # Every selected role must use the same role map containing only its PTY.
        assert all(roles[role]['port'].startswith('/dev/pts/') for role in selected)
        assert len({roles[role]['port'] for role in selected}) == len(selected)
        for role in selected:
            session = GravitySession(roles)
            sessions[role] = session
            histories.append(session)
            session.start(role, mock=False, with_rviz=False)
        for role, session in sessions.items():
            wait(session, lambda state: state['ready'], timeout=35)
            emulator = emulators[role]
            motor = emulator.snapshot()
            assert not motor['commands'] and not motor['enabled'] and not motor['nonqueries']
        assert len({s.namespace for s in sessions.values()}) == len(selected)
        assert len({id(s.stack.context) for s in sessions.values()}) == len(selected)
        stages.append('parallel_feedback_readonly')
        print('PASS: simultaneous feedback connections remain read-only', flush=True)

        for session in sessions.values():
            session.enable()
        for role, session in sessions.items():
            wait(session, lambda state: state['active'] and not state['busy'] and
                 state.get('commanded') is not None, timeout=35)
            wait(session, lambda state: emulators[role].snapshot()['commands'] > 100)
        stages.append('parallel_explicit_enable')
        print('PASS: simultaneous explicit enable; separate contexts and namespaces', flush=True)

        changed_role = selected[0]
        changed_session = sessions[changed_role]
        monitor = ContinuityMonitor({role: (session, emulators[role])
                                     for role, session in sessions.items() if role != changed_role})
        monitor.worker.start()

        duplicate = GravitySession(roles)
        histories.append(duplicate)
        duplicate.start(changed_role, mock=False, with_rviz=False)
        wait(duplicate, lambda state: not state['running'], timeout=10, allow_error=True)
        duplicate.worker.join(3)
        assert '占用' in duplicate.snapshot()['error'], duplicate.snapshot()
        assert duplicate.stack is None, 'duplicate arm must fail before starting native processes'
        assert changed_session.snapshot()['active'] and changed_session._fresh(enabled=True)
        monitor.check()
        stages.append('duplicate_usb_rejected')
        print('PASS: second session for the same USB is rejected before opening it', flush=True)

        original = np.array(changed_session.snapshot()['commanded'])
        changed = deepcopy(changed_session.snapshot()['applied_parameters'])
        index = int(np.argmax(np.abs(original[:6])))
        assert abs(original[index]) > .1, 'test posture must produce measurable gravity torque'
        changed['torque_scales'][index] = 0.
        changed_session.tune(changed)
        wait(changed_session, lambda state: not state['busy'] and state['tuning_revision'] == 1)
        wait(changed_session, lambda state: abs(state['commanded'][index] - original[index]) > .1)
        monitor.check()
        stages.append('tuning_isolated')
        print('PASS: live tuning changes only the selected arm', flush=True)

        changed_session.release()
        wait(changed_session, lambda state: state['release_confirmed'] and not state['busy'])
        assert not emulators[changed_role].snapshot()['enabled']
        changed_session.stop()
        changed_session.worker.join(20)
        assert not changed_session.worker.is_alive() and not changed_session.snapshot()['error']
        assert all(child.poll() is not None for _, child in changed_session.stack.children)
        monitor.check()
        replacement = GravitySession(roles)
        histories.append(replacement)
        sessions[changed_role] = replacement
        replacement.start(changed_role, mock=False, with_rviz=False)
        wait(replacement, lambda state: state['ready'], timeout=35)
        assert not emulators[changed_role].snapshot()['enabled']
        replacement.enable()
        wait(replacement, lambda state: state['active'] and not state['busy'], timeout=35)
        monitor.check()
        stages.append('release_disconnect_reconnect_isolated')
        print('PASS: one arm releases and reconnects while the others keep compensating', flush=True)

        emulators[changed_role].snapshot('drop_feedback')
        wait(replacement, lambda state: bool(state['error']), timeout=8, allow_error=True)
        wait(replacement, lambda state: not state['running'], timeout=25, allow_error=True)
        replacement.worker.join(5)
        assert not replacement.worker.is_alive()
        assert not emulators[changed_role].snapshot()['enabled'], 'faulted arm must receive disable'
        assert replacement.snapshot()['release_confirmed'] is False
        monitor.close()
        stages.append('feedback_fault_isolated')
        print('PASS: feedback loss disables only the failed arm; peer feedback/output remain healthy', flush=True)
        result.update(result='PASS', peer_monitor_samples=monitor.samples,
                      observer_resyncs=monitor.observer_resyncs,
                      max_observer_resync_seconds=monitor.max_observer_resync_seconds,
                      no_peer_enable_disable_during_other_arm_operations=True,
                      transport={role: em.port for role, em in emulators.items()},
                      reports=[session.snapshot()['directory'] for session in histories],
                      fault_error=replacement.snapshot()['error'])
    finally:
        if monitor:
            monitor.finished.set()
            monitor.worker.join(3)
        cleanup(histories)
        final_motors = {role: emulator.snapshot() for role, emulator in emulators.items()}
        for emulator in emulators.values():
            motor = emulator.snapshot()
            assert not motor['enabled'], 'cleanup left an emulated motor enabled'
            assert not motor['zero_commands']
            assert not motor['gripper_commands'], 'gripper must remain read-only'
            emulator.close()
    result.update(all_children_exited=all(not session.stack or
                  all(child.poll() is not None for _, child in session.stack.children)
                  for session in histories),
                  elapsed_seconds=round(time.monotonic() - started, 3),
                  mit_commands={role: motor['commands'] for role, motor in final_motors.items()})
    assert result['all_children_exited']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roles', choices=ROLE_ORDER, nargs='+', default=['teaching_left', 'teaching_right'])
    parser.add_argument('--output', type=Path, default=ROOT.parent/'diagnostics/gravity_multi_revision_20261125/multi_backend.json')
    args = parser.parse_args()
    if not os.environ.get('CYCLONEDDS_URI'):
        os.environ['CYCLONEDDS_URI'] = 'file://' + str((ROOT/'scripts/gravity_dds.xml').resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = verify(args.roles)
    except BaseException as exc:
        args.output.write_text(json.dumps(dict(result='FAIL', roles=args.roles,
                              physical_devices=0, error=repr(exc)), ensure_ascii=False, indent=2))
        raise
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
