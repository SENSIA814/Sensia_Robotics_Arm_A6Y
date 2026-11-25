"""A5 URDF FK and bounded damped differential IK, in joint coordinates."""
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from scipy.optimize import lsq_linear
import yaml

JOINTS = [f'joint{i}' for i in range(1, 7)] + ['gripper_finger_joint1']


class A5Kinematics:
    def __init__(self, urdf, limits_file, config):
        self.model = pin.buildModelFromUrdf(str(urdf), True)
        if self.model.nq != 7 or self.model.nv != 7:
            raise ValueError('A5 requires native mimic support and seven independent coordinates')
        self.data = self.model.createData()
        self.frame = self.model.getFrameId('tool_tcp')
        if self.frame >= self.model.nframes:
            raise ValueError('tool_tcp missing')
        ids = [self.model.getJointId(n) for n in JOINTS]
        if any(i == 0 for i in ids):
            raise ValueError('independent joint missing')
        self.qidx = [self.model.idx_qs[i] for i in ids]
        self.vidx = [self.model.idx_vs[i] for i in ids]
        if len(set(self.qidx)) != 7 or any(self.model.nqs[i] != 1 for i in ids):
            raise ValueError('invalid independent mapping')
        runtime = yaml.safe_load(open(limits_file))['joint_limits']
        root = ET.parse(urdf).getroot()
        cad = [root.find(f"joint[@name='{n}']/limit") for n in JOINTS]
        self.lower = np.array([max(float(c.get('lower')), runtime[n]['min_position']) for c,n in zip(cad,JOINTS)])
        self.upper = np.array([min(float(c.get('upper')), runtime[n]['max_position']) for c,n in zip(cad,JOINTS)])
        self.zero_tolerance = np.array([runtime[n].get('zero_tolerance', 0.) for n in JOINTS], dtype=float)
        if self.zero_tolerance.shape != (7,) or not np.isfinite(self.zero_tolerance).all() or np.any(self.zero_tolerance < 0):
            raise ValueError('invalid zero tolerance')
        self.feedback_lower = self.lower - np.where(self.lower == 0., self.zero_tolerance, 0.)
        self.feedback_upper = self.upper + np.where(self.upper == 0., self.zero_tolerance, 0.)
        requested = np.asarray(config['max_velocity'], dtype=float)
        if requested.shape != (7,) or not np.isfinite(requested).all() or np.any(requested <= 0):
            raise ValueError('invalid velocity limits')
        self.speed = np.minimum(requested, [runtime[n]['max_velocity'] for n in JOINTS])
        if np.any(self.lower >= self.upper) or np.any(self.speed <= 0):
            raise ValueError('inconsistent runtime limits')
        self.weights = np.array([1,1,1] + [float(config['orientation_length_scale'])]*3)
        if type(config.get('bounded_ik', False)) is not bool:
            raise ValueError('invalid bounded_ik')
        self.bounded_ik = config.get('bounded_ik', False)
        self.last_tracking_limited = False

    def configuration(self, q):
        full = np.zeros(7)
        full[self.qidx] = q
        return full

    def fk(self, q):
        pin.framesForwardKinematics(self.model, self.data, self.configuration(q))
        return self.data.oMf[self.frame].homogeneous.copy()

    def error(self, q, target):
        pose = self.fk(q)
        return np.r_[target[:3,3]-pose[:3,3], pin.log3(target[:3,:3] @ pose[:3,:3].T)]

    def jacobian(self, q):
        return pin.computeFrameJacobian(self.model, self.data, self.configuration(q),
            self.frame, pin.LOCAL_WORLD_ALIGNED)[:, self.vidx[:6]].copy()

    def feedback_in_bounds(self, q):
        """Accept measured zero error without altering any measured coordinate."""
        q = np.asarray(q)
        return bool(q.shape == (7,) and np.isfinite(q).all() and
                    np.all(q >= self.feedback_lower) and np.all(q <= self.feedback_upper))

    def reference_bounds(self, q, allow_zero_tolerance=False):
        q = np.asarray(q)
        if not self.feedback_in_bounds(q):
            raise ValueError('seed outside feedback limits')
        if allow_zero_tolerance:
            # An accepted measured anchor outside the nominal range may hold or
            # move inward. Recompute at each step; never command farther outward.
            return np.minimum(self.lower, q), np.maximum(self.upper, q)
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('seed outside nominal limits')
        return self.lower, self.upper

    def step(self, q, target, dt, allow_zero_tolerance=False,
             tracking_position=None, tracking_limit=None):
        self.last_tracking_limited = False
        if not np.isfinite(target).all() or not np.isfinite(dt) or dt <= 0:
            raise ValueError('invalid IK target or timestep')
        lower, upper = self.reference_bounds(q, allow_zero_tolerance)
        J = self.weights[:,None] * self.jacobian(q)
        e = self.weights * self.error(q, target)
        sigma = float(np.linalg.svd(J, compute_uv=False)[-1])
        damping = 0.001 + max(0., 0.025-sigma) * 0.3
        bound = self.speed[:6] * dt
        lo = np.maximum(lower[:6], q[:6]-bound)
        hi = np.minimum(upper[:6], q[:6]+bound)
        tracking_lo = tracking_hi = None
        if self.bounded_ik:
            if tracking_position is not None:
                measured = np.asarray(tracking_position, dtype=float)
                if (measured.shape != (6,) or not np.isfinite(measured).all() or
                        tracking_limit is None or not np.isfinite(tracking_limit) or tracking_limit <= 0):
                    raise ValueError('invalid tracking bounds')
                # Each axis gets its own remaining lead budget. An existing
                # out-of-band reference may hold or move inward, never farther out.
                tracking_lo = np.minimum(q[:6], measured-tracking_limit)
                tracking_hi = np.maximum(q[:6], measured+tracking_limit)
                lo = np.maximum(lo, tracking_lo)
                hi = np.minimum(hi, tracking_hi)
            # Solve with the bounds, rather than clipping an unconstrained solve
            # or shrinking every joint because a single slow joint has no budget.
            lower_step, upper_step = lo-q[:6], hi-q[:6]
            free = upper_step-lower_step > 1e-12
            delta = np.zeros(6)
            if np.any(free):
                matrix = np.vstack((J, damping*np.eye(6)))
                result = lsq_linear(matrix[:, free], np.r_[e, np.zeros(6)],
                    bounds=(lower_step[free], upper_step[free]), method='bvls', tol=1e-10, max_iter=30)
                if not result.success:
                    raise ValueError('bounded IK did not converge')
                delta[free] = result.x
        else:
            delta = np.linalg.solve(J.T @ J + damping**2*np.eye(6), J.T @ e)
            # Preserve the legacy policy in virtual/trial/impedance modes.
            delta /= max(1., float(np.max(np.abs(delta)/bound)))
        if not np.isfinite(delta).all():
            raise ValueError('nonfinite IK result')
        best = q.copy()
        for factor in (1., .5, .25, .125):
            candidate = q.copy()
            candidate[:6] = np.clip(q[:6]+factor*delta, lo, hi)
            if np.linalg.norm(self.weights*self.error(candidate,target)) < np.linalg.norm(e) + 1e-12:
                best = candidate
                break
        if tracking_lo is not None:
            self.last_tracking_limited = bool(np.any(
                ((tracking_lo > np.maximum(lower[:6], q[:6]-bound)+1e-12) &
                 (best[:6] <= tracking_lo+1e-10)) |
                ((tracking_hi < np.minimum(upper[:6], q[:6]+bound)-1e-12) &
                 (best[:6] >= tracking_hi-1e-10))))
        return best, sigma
