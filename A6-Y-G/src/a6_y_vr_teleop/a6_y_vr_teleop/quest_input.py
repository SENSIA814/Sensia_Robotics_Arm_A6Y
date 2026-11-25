"""Decode Vuer 0.0.70 right controller events without ROS or Piper imports."""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .input_protocol import make_packet


class RightControllerDecoder:
    def __init__(self, session_id):
        self.session = session_id
        self.sequence = self.pose_sequence = self.epoch = 0
        self.pose_ns = self.buttons_ns = 0
        self.position = [0.,0.,0.]
        self.quaternion = [0.,0.,0.,1.]
        self.squeeze = self.button_a = False
        self.trigger = .5
        self.pose_valid = self.buttons_valid = False

    def snapshot(self, now_ns, connected=True):
        return make_packet(self.session,self.sequence,now_ns,self.position,self.quaternion,
            self.squeeze,self.trigger,self.epoch,pose_sequence=self.pose_sequence,
            pose_received_monotonic_ns=self.pose_ns,buttons_received_monotonic_ns=self.buttons_ns,
            button_a=self.button_a,pose_valid=self.pose_valid,buttons_valid=self.buttons_valid,
            connected=connected)

    def update(self, value, now_ns):
        if not isinstance(value,dict) or not ('right' in value or 'rightState' in value):
            return None
        self.sequence += 1
        if 'right' in value:
            self.pose_valid=False
            try:
                raw=np.asarray(value['right'],dtype=float)
                if raw.shape != (16,): raise ValueError('expected column-major matrix')
                matrix=raw.reshape(4,4,order='F')
                R=matrix[:3,:3]
                if not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1],atol=1e-5):
                    raise ValueError('invalid homogeneous matrix')
                if not np.allclose(R.T@R,np.eye(3),atol=1e-3) or not np.isclose(np.linalg.det(R),1,atol=1e-3):
                    raise ValueError('invalid rotation')
                q=Rotation.from_matrix(R).as_quat()
                if np.dot(q,self.quaternion)<0: q=-q
                self.position=matrix[:3,3].tolist();self.quaternion=q.tolist()
                self.pose_ns=now_ns;self.pose_sequence+=1;self.pose_valid=True
            except (TypeError,ValueError):
                pass
        rs=value.get('rightState')
        if rs is not None:
            self.buttons_valid=False
            try:
                if not isinstance(rs,dict) or type(rs['squeeze']) is not bool:
                    raise ValueError('missing grip state')
                trigger=float(rs.get('triggerValue',float(rs.get('trigger',False))))
                a=rs.get('aButton',False)
                if not math.isfinite(trigger) or not 0<=trigger<=1 or type(a) is not bool:
                    raise ValueError('invalid buttons')
                if rs['squeeze'] != self.squeeze: self.epoch+=1
                self.squeeze=rs['squeeze'];self.trigger=trigger;self.button_a=a
                self.buttons_ns=now_ns;self.buttons_valid=True
            except (KeyError,TypeError,ValueError):
                pass
        return self.snapshot(now_ns)

    def disconnect(self, now_ns):
        self.sequence+=1
        self.pose_valid=False
        return self.snapshot(now_ns,connected=False)
