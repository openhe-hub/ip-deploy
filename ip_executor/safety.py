"""Safety leash for IP commanded EE poses.

Mirrors the heuristic used in teleop_ps4.py: the commanded target is clamped so
its translation is at most `max_pos_step_m` ahead of the measured pose, and its
rotation is at most `max_rot_step_rad` ahead.

Linear interp is applied on the slerp path for orientation when needed.
"""

from __future__ import annotations

import math

import numpy as np


def quat_xyzw_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y+z*z),     s*(x*y-z*w),       s*(x*z+y*w)],
        [    s*(x*y+z*w), 1 - s*(x*x+z*z),       s*(y*z-x*w)],
        [    s*(x*z-y*w),     s*(y*z+x*w),   1 - s*(x*x+y*y)],
    ])


def R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return np.array([(m[2, 1]-m[1, 2])*s, (m[0, 2]-m[2, 0])*s,
                         (m[1, 0]-m[0, 1])*s, 0.25/s])
    if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return np.array([0.25*s, (m[0, 1]+m[1, 0])/s, (m[0, 2]+m[2, 0])/s,
                         (m[2, 1]-m[1, 2])/s])
    if m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        return np.array([(m[0, 1]+m[1, 0])/s, 0.25*s, (m[1, 2]+m[2, 1])/s,
                         (m[0, 2]-m[2, 0])/s])
    s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
    return np.array([(m[0, 2]+m[2, 0])/s, (m[1, 2]+m[2, 1])/s, 0.25*s,
                     (m[1, 0]-m[0, 1])/s])


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta_0 * t
    sin_theta_0 = math.sin(theta_0)
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return s0 * q0 + s1 * q1


def angle_between_quats_xyzw(q0: np.ndarray, q1: np.ndarray) -> float:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = abs(float(np.dot(q0, q1)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def leash(measured_pos: np.ndarray, measured_quat_xyzw: np.ndarray,
          target_pos: np.ndarray, target_quat_xyzw: np.ndarray,
          max_pos_step_m: float = 0.04,
          max_rot_step_rad: float = math.radians(17.0)
          ) -> tuple[np.ndarray, np.ndarray]:
    """Clamp (target - measured) translation and rotation deltas."""
    dp = target_pos - measured_pos
    n = float(np.linalg.norm(dp))
    if n > max_pos_step_m and n > 0:
        dp = dp * (max_pos_step_m / n)
    pos_out = measured_pos + dp

    ang = angle_between_quats_xyzw(measured_quat_xyzw, target_quat_xyzw)
    if ang > max_rot_step_rad and ang > 0:
        t = max_rot_step_rad / ang
        quat_out = slerp(measured_quat_xyzw, target_quat_xyzw, t)
    else:
        quat_out = target_quat_xyzw / np.linalg.norm(target_quat_xyzw)
    return pos_out.astype(np.float32, copy=False), quat_out.astype(np.float32, copy=False)
