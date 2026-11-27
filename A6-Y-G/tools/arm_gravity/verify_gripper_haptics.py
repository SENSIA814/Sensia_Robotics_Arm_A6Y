#!/usr/bin/env python3
"""Real ROS/native driver contact + release test on PTYs; no physical devices."""
import json
from pathlib import Path
import time
from gravity_config import ROOT, load_roles
from teaching_port import TeachingCoordinator
from teaching_gripper import gripper_mapping
from verify_multi import ProcessMotorEmulator, cleanup
from verify_teaching_port import wait


def verify():
    emulators={}; c=None
    leader, follower='teaching_left','motion_left'
    try:
        roles=load_roles()
        for role in (leader,follower):
            emulators[role]=ProcessMotorEmulator(role,position_mode=role==follower,native_port=True)
            emulators[role].snapshot(('set_q',[0.]*6+[-.025]))
            roles[role]=dict(roles[role],port=emulators[role].port)
            assert roles[role]['port'].startswith('/dev/pts/')
        c=TeachingCoordinator(roles=roles,report_dir=ROOT.parent/'diagnostics/arm_teaching_test_reports')
        c.start([(leader,follower)],mock=False,with_rviz=False)
        state=wait(c,lambda s:s['phase']=='following',timeout=35)
        time.sleep(1.6)
        mapping=gripper_mapping(c.sessions[leader].model,c.sessions[follower].model)
        def hand(target):
            emulators[leader].snapshot(('set_q',[0.]*6+[(target-mapping['offset'])/mapping['scale']]))
        free=emulators[leader].snapshot()['gripper_torque_history']
        assert free and max(abs(v) for _,v in free)<.0025
        damping={}
        for velocity in (.0239,-.0239,0.):
            emulators[leader].snapshot(('set_dq',[0.]*6+[velocity]));time.sleep(.12)
            torque=emulators[leader].snapshot()['last_torque'][7]
            damping[velocity]=torque
            assert abs(torque)<.0175
            if velocity:assert torque*velocity<0
            else:assert abs(torque)<.0025
        # Strong imposed load exercises the requested 2.0 Nm teacher cap.
        emulators[follower].snapshot(('contact_j7',-.020,4.0))
        hand(-.012)
        state=wait(c,lambda s:s.get('gripper_haptics',{}).get(follower)=='contact',timeout=4.)
        time.sleep(3.0)  # Allow the unchanged 0.8 Nm/s ramp to reach the new cap.
        held=emulators[follower].snapshot()['last_position'][7]*.0239
        assert -.02005 <= held <= -.01985,held
        resisting=emulators[leader].snapshot()['last_torque'][7]
        assert -2.000001 <= resisting < -1.99,resisting
        hand(-.004);time.sleep(.4)
        assert abs(emulators[follower].snapshot()['last_position'][7]*.0239-held)<.00001
        hand(-.005);time.sleep(.2)
        state=wait(c,lambda s:s.get('gripper_haptics',{}).get(follower)=='free',timeout=2.)
        opened=emulators[follower].snapshot()['q'][6]
        assert opened < held-.0002,(opened,held)
        assert abs(emulators[leader].snapshot()['last_torque'][7])<.0025
        # Re-contact, then lose the follower. No stale resistance may persist.
        hand(-.0008)
        wait(c,lambda s:s.get('gripper_haptics',{}).get(follower)=='contact',timeout=4.)
        time.sleep(.15)
        emulators[follower].snapshot('drop_feedback')
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:
            audit=emulators[leader].snapshot()
            if abs(audit['last_torque'][7])<.0025:break
            time.sleep(.005)
        assert abs(audit['last_torque'][7])<.0025,'Stale haptic torque persisted'
        emulators[follower].snapshot('restore_feedback')
        c.release()
        wait(c,lambda s:s['phase'] in ('released','error'),timeout=35,allow_error=True)
        result=dict(result='PASS',physical_devices=0,report=state['directory'],
                    stationary_zero_torque=True,free_damping_motor_Nm=damping,
                    contact_target_m=held,reflected_motor_Nm=resisting,
                    continued_squeeze_did_not_close=True,opening_released_immediately=True,
                    released_position_m=opened,feedback_loss_cleared_torque=True,
                    plant='ideal encoder with imposed unilateral J7 stop; no mechanical validation')
        Path(state['directory'],'verification_haptics.json').write_text(json.dumps(result,indent=2))
        return result
    finally:
        if c:
            cleanup(list(c.sessions.values()))
            if c.worker:c.worker.join(10)
        for em in emulators.values():em.close()


if __name__=='__main__':print(json.dumps(verify(),indent=2))
