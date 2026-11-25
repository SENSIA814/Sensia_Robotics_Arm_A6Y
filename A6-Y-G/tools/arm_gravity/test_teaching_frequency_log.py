import csv
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from gravity_config import JOINTS
from teaching_frequency_log import FrequencyLog


class FrequencyLogTests(unittest.TestCase):
    def test_driver_counts_survive_observer_gaps_and_reset_without_false_rates(self):
        sample={'rows':{}}
        session=SimpleNamespace(stack=SimpleNamespace(manager_name='cm1'),
                                feedback=SimpleNamespace(snapshot=lambda:sample))
        def update(t, count, dry_run=0):
            sample['rows']={name:dict(control_time=t,control_cycle_count=count,
                control_overrun_count=2,control_period=.004,read_duration=.0002,
                write_duration=.0001,bus_cycle_duration=.0003,dry_run=dry_run,
                command_count=count if i<6 else 0,feedback_count=count,
                feedback_age=.004) for i,name in enumerate(JOINTS)}
        with tempfile.TemporaryDirectory() as temp:
            log=FrequencyLog(temp)
            update(100.,1000);log.sample({'arm':session},'following')
            # 250 intervening cycles, despite no intermediate observer samples.
            update(101.,1250);log.sample({'arm':session},'following')
            log.sample({'arm':session},'following')  # unchanged stale state
            update(102.,10);log.sample({'arm':session},'following')  # reset
            session.stack.manager_name='cm2'
            update(103.,300);log.sample({'arm':session},'following')
            update(104.,550,dry_run=1);log.sample({'arm':session},'following')
            log.close()
            with (Path(temp)/'frequency.csv').open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(float(rows[1]['control_hz']),250.)
        self.assertEqual(float(rows[1]['command_hz_j1']),250.)
        self.assertEqual(float(rows[1]['command_hz_j7']),0.)
        self.assertEqual(float(rows[1]['feedback_hz_j6']),250.)
        self.assertEqual(float(rows[1]['feedback_age_ms_j1']),4.)
        for i in (0,2,3,4):self.assertEqual(rows[i]['control_hz'],'')
        self.assertEqual(rows[5]['command_hz_j1'],'')


if __name__=='__main__':unittest.main()
