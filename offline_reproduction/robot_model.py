from __future__ import annotations

import numpy as np


def wrap_angle(angle: np.ndarray | float) -> np.ndarray | float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rollout_unicycle(
    pose: np.ndarray,
    velocities: np.ndarray,
    angular_velocities: np.ndarray,
    horizon_s: float,
    integration_dt_s: float,
) -> np.ndarray:
    steps = max(1, int(round(horizon_s / integration_dt_s)))
    trajectories = np.empty((len(velocities), steps, 3), dtype=np.float32)
    x = np.full(len(velocities), float(pose[0]), dtype=np.float32)
    y = np.full(len(velocities), float(pose[1]), dtype=np.float32)
    theta = np.full(len(velocities), float(pose[2]), dtype=np.float32)
    for index in range(steps):
        x = x + velocities * np.cos(theta) * integration_dt_s
        y = y + velocities * np.sin(theta) * integration_dt_s
        theta = wrap_angle(theta + angular_velocities * integration_dt_s)
        trajectories[:, index, 0] = x
        trajectories[:, index, 1] = y
        trajectories[:, index, 2] = theta
    return trajectories


def integrate_unicycle(pose: np.ndarray, velocity: float, omega: float, dt_s: float) -> np.ndarray:
    x, y, theta = (float(v) for v in pose)
    return np.asarray(
        [
            x + velocity * np.cos(theta) * dt_s,
            y + velocity * np.sin(theta) * dt_s,
            wrap_angle(theta + omega * dt_s),
        ],
        dtype=float,
    )

