"""Geometry conversion only; no inverse kinematics, planning or device access."""
import numpy as np
import pinocchio as pin


def pose_message(transform):
    from geometry_msgs.msg import Pose
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, transform[:3, 3])
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, pin.Quaternion(transform[:3, :3]).coeffs())
    return result


def pose_matrix(pose):
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(np.r_[q, xyz]).all() or np.linalg.norm(q) < .001:
        raise ValueError('目标位姿无效')
    q /= np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = pin.Quaternion(q[3], q[0], q[1], q[2]).matrix()
    result[:3, 3] = xyz
    return result
