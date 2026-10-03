"""
common/pose.py
==============
Local pose + odometry model.

The Writer/Executor never read their position from the world. They keep a
pose (x, y, heading) in their OWN local frame (origin = zone entry, +x =
direction the robot faced at entry, +y = to its left, heading in radians
counter-clockwise from +x) and update it by integrating motion commands:

    turn(delta)  -> heading += delta            (gyro/IMU on the real robot)
    forward(d)   -> x += d cos(h), y += d sin(h)  (wheel encoders)

Optional Gaussian noise models odometry error. It is OFF by default so the
test-suite is deterministic. [SIMULATED] — no real odometry error has been
measured; a mathematical model says nothing about real localisation accuracy.

[IMPLEMENTED]
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


@dataclass
class LocalPose:
    """Pose in the robot's local frame. Units: metres, radians."""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0

    def copy(self) -> "LocalPose":
        return LocalPose(self.x, self.y, self.heading)


class OdometryModel:
    """Integrates turn/forward increments into a LocalPose."""

    def __init__(self, pose: Optional[LocalPose] = None, *,
                 dist_noise_std: float = 0.0, turn_noise_std: float = 0.0, seed: int = 0) -> None:
        self.pose = pose or LocalPose()
        self.dist_noise_std = dist_noise_std
        self.turn_noise_std = turn_noise_std
        self._rng = random.Random(seed)
        self.path_length_m = 0.0
        self.turn_count = 0

    def turn(self, delta_rad: float) -> float:
        """Rotate by delta (CCW positive = a LEFT turn). Returns the delta applied."""
        if delta_rad == 0.0:
            return 0.0
        applied = delta_rad
        if self.turn_noise_std > 0:
            applied += self._rng.gauss(0.0, self.turn_noise_std)
        self.pose.heading = wrap_angle(self.pose.heading + applied)
        self.turn_count += 1
        return applied

    def turn_to(self, heading_rad: float) -> float:
        """Turn the shortest way to an absolute heading."""
        return self.turn(wrap_angle(heading_rad - self.pose.heading))

    def forward(self, dist_m: float) -> None:
        d = dist_m
        if self.dist_noise_std > 0:
            d += self._rng.gauss(0.0, self.dist_noise_std)
        self.pose.x += d * math.cos(self.pose.heading)
        self.pose.y += d * math.sin(self.pose.heading)
        self.path_length_m += abs(d)

    def move_by(self, dx_m: float, dy_m: float) -> None:
        """Turn toward (dx,dy) then drive its length — one grid step."""
        dist = math.hypot(dx_m, dy_m)
        if dist == 0.0:
            return
        self.turn_to(math.atan2(dy_m, dx_m))
        self.forward(dist)
