"""Relative WebXR-to-base mapping shared by virtual and physical preview."""
import numpy as np
import pinocchio as pin


def relative_target(anchor_tcp, anchor_frame, frame, basis, scale, rotation=True):
    target = anchor_tcp.copy()
    target[:3, 3] += scale * basis @ (frame.position-anchor_frame.position)
    if rotation:
        target[:3, :3] = basis @ frame.rotation @ anchor_frame.rotation.T @ basis.T @ anchor_tcp[:3, :3]
    return target


def limit_target(previous, desired, dt, linear, angular):
    target = previous.copy()
    delta = desired[:3, 3]-previous[:3, 3]
    target[:3, 3] += delta * min(1., linear*dt/max(np.linalg.norm(delta), 1e-12))
    rot = pin.log3(desired[:3, :3] @ previous[:3, :3].T)
    rot *= min(1., angular*dt/max(np.linalg.norm(rot), 1e-12))
    target[:3, :3] = pin.exp3(rot) @ previous[:3, :3]
    return target
