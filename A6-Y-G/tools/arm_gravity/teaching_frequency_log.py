"""Low-rate disk logging of cumulative native-driver counters, outside RT control."""
import csv
import math
from pathlib import Path
import time

from gravity_config import JOINTS


class FrequencyLog:
    def __init__(self, folder):
        self.previous = {}
        self.file = (Path(folder)/'frequency.csv').open('w', newline='')
        self.fields = ['wall_time', 'phase', 'role', 'manager', 'dry_run', 'diagnostic_age_s',
                       'control_time', 'control_cycle_count', 'control_period_ms', 'control_hz',
                       'control_overrun_count', 'overruns_since_previous',
                       'read_ms', 'write_ms', 'bus_cycle_ms']
        self.fields += [f'{kind}_j{i}' for kind in
                        ('command_count', 'feedback_count', 'command_hz', 'feedback_hz', 'feedback_age_ms')
                        for i in range(1, 8)]
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()

    def sample(self, sessions, phase):
        for role, session in sessions.items():
            sample = session.feedback.snapshot()
            rows = [sample.get('rows', {}).get(name, {}) for name in JOINTS]
            first = rows[0]
            stamp = first.get('control_time')
            manager = session.stack.manager_name
            record = dict(wall_time=time.time(), phase=phase, role=role, manager=manager,
                          dry_run=first.get('dry_run'), control_time=stamp,
                          control_cycle_count=first.get('control_cycle_count'),
                          control_overrun_count=first.get('control_overrun_count'))
            received = sample.get('diagnostic_time')
            record['diagnostic_age_s'] = max(0., time.monotonic()-received) if received else None
            for output, source in (('control_period_ms','control_period'), ('read_ms','read_duration'),
                                   ('write_ms','write_duration'), ('bus_cycle_ms','bus_cycle_duration')):
                value = first.get(source)
                record[output] = value*1000 if value is not None and value >= 0 else None
            for i, row in enumerate(rows, 1):
                for kind in ('command_count', 'feedback_count'):
                    record[f'{kind}_j{i}'] = row.get(kind)
                age = row.get('feedback_age')
                record[f'feedback_age_ms_j{i}'] = age*1000 if age is not None and age >= 0 else None

            # Driver monotonic time and counters share one dynamic-state message.
            # Do not use observer callback/wall-clock spacing as a CAN frequency.
            previous = self.previous.get(role)
            valid = stamp is not None and math.isfinite(stamp) and stamp > 0
            if valid and previous and manager == previous['manager'] and stamp > previous['control_time']:
                dt = stamp-previous['control_time']
                def delta(key):
                    a, b = record.get(key), previous.get(key)
                    return a-b if a is not None and b is not None and a >= b else None
                cycles = delta('control_cycle_count')
                if cycles is not None:
                    record['control_hz'] = cycles/dt
                    record['overruns_since_previous'] = delta('control_overrun_count')
                    # dry_run does not transmit or receive physical/SLCAN frames.
                    if first.get('dry_run') == 0:
                        for kind in ('command', 'feedback'):
                            for i in range(1, 8):
                                count = delta(f'{kind}_count_j{i}')
                                record[f'{kind}_hz_j{i}'] = count/dt if count is not None else None
            if valid:
                self.previous[role] = record
            self.writer.writerow(record)
        self.file.flush()

    def close(self):
        self.file.close()
