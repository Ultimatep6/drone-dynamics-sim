from dataclasses import dataclass

import numpy as np
from numpy import cos as c
from numpy import sin as s

from quad_sim.math.safe_trig import asin as As
from quad_sim.math.safe_trig import atan2 as At


@dataclass
class Eulerian:
    """Euler angles in ZYX convention: yaw, pitch, roll."""

    yaw: float
    pitch: float
    roll: float

    # -----------------------------
    # Utilities
    # -----------------------------
    def as_np(self) -> np.ndarray:
        """Return as (3,1) numpy column vector."""
        return np.array([[self.yaw], [self.pitch], [self.roll]], dtype=float)

    @staticmethod
    def from_np(arr: np.ndarray):
        """Construct from (3,1) numpy array."""
        if not isinstance(arr, np.ndarray):
            raise TypeError("Eulerian.from_np expects a numpy array")
        if arr.shape != (3, 1):
            raise TypeError("Eulerian.from_np expects shape (3,1)")
        yaw, pitch, roll = arr[:, 0]
        return Eulerian(yaw, pitch, roll)

    # -----------------------------
    # Euler → Quaternion
    # -----------------------------
    def to_quaternion(self):
        """Convert ZYX yaw-pitch-roll to quaternion."""
        cy, sy = c(self.yaw / 2), s(self.yaw / 2)
        cp, sp = c(self.pitch / 2), s(self.pitch / 2)
        cr, sr = c(self.roll / 2), s(self.roll / 2)

        from quad_sim.math.quaternion import Quaternion  # avoid circular import

        return Quaternion(
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
        )

    def __str__(self) -> str:
        return f"Roll: {self.roll}\nPitch: {self.pitch}\nYaw: {self.yaw}"
