from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy import cos as c
from numpy import sin as s

from quad_sim.funcs import asin as As
from quad_sim.funcs import atan2 as At


@dataclass
class Quaternion:
    """Quaternion in (w, x, y, z) format with ZYX yaw-pitch-roll convention."""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # -----------------------------
    # Utilities
    # -----------------------------
    def as_np(self) -> np.ndarray:
        return np.array([[self.w, self.x, self.y, self.z]], dtype=float).T

    def norm(self) -> float:
        return float(np.linalg.norm(self.as_np()))

    def normalized(self):
        n = self.norm()
        if n == 0:
            raise ZeroDivisionError("Cannot normalize a zero quaternion")

        # .flatten() or .ravel() turns (4, 1) into (4,)
        q = (self.as_np() / n).flatten()

        return Quaternion(*q)

    # -----------------------------
    # Conjugate / Inverse
    # -----------------------------
    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def inverse(self):
        n = self.norm()
        if not np.isclose(n, 1.0, atol=1e-12):
            raise ValueError(
                f"Quaternion must be unit-normalized for inverse, got norm={n}"
            )
        return self.conjugate()

    # -----------------------------
    # Hamilton product // Change of quaternion orientation
    # -----------------------------
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other)
        if not isinstance(other, Quaternion):
            return NotImplemented

        # Use .flatten() or .ravel() to turn (4,1) into (4,)
        w1, x1, y1, z1 = self.as_np().flatten()
        w2, x2, y2, z2 = other.as_np().flatten()

        return Quaternion(
            float(w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2),
            float(w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2),
            float(w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2),
            float(w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2),
        )
    
    def __add__(self, other):
        if not isinstance(other, Quaternion):
            return NotImplemented

        w1, x1, y1, z1 = self.as_np().flatten()
        w2, x2, y2, z2 = other.as_np().flatten()

        return Quaternion(
            float(w1 + w2),
            float(x1 + x2),
            float(y1 + y2),
            float(z1 + z2),
        )

    # -----------------------------
    # Quaternion → Euler
    # -----------------------------
    def to_euler(self):
        n = self.norm()
        if not np.isclose(n, 1.0, atol=1e-12):
            raise ValueError("Quaternion must be unit-normalized")

        w, x, y, z = self.as_np().flatten()

        # yaw (Z axis rotation)
        yaw = At(2 * (w * z + x * y), w * w + x * x - y * y - z * z)

        # pitch (Y axis rotation)
        pitch = As(2 * (w * y - x * z))

        # roll (X axis rotation)
        roll = At(2 * (w * x + y * z), w * w - x * x - y * y + z * z)

        from quad_sim.orientation.eulerian import Eulerian  # avoid circular import

        return Eulerian(yaw, pitch, roll)

    # -----------------------------
    # Euler → Quaternion
    # -----------------------------
    @classmethod
    def from_euler(cls, objB: "Eulerian"):
        from quad_sim.orientation.eulerian import Eulerian  # avoid circular import

        if not isinstance(objB, Eulerian):
            raise TypeError("from_euler expects an Eulerian object")

        yaw = objB.yaw
        pitch = objB.pitch
        roll = objB.roll

        cy, sy = c(yaw / 2), s(yaw / 2)
        cp, sp = c(pitch / 2), s(pitch / 2)
        cr, sr = c(roll / 2), s(roll / 2)

        return cls(
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
        )

    @classmethod
    def unpackArray(cls, arr: np.ndarray):
        if not (isinstance(arr, np.ndarray) and arr.shape == (4, 1)):
            raise ValueError("arr must be a numpy.ndarray with shape (4,1)")

        q = cls(*(arr.T.flatten().tolist()))
        if not np.isclose(q.norm(), 1.0, atol=1e-12):
            q = q.normalized()

        return q

    def __str__(self) -> str:
        return f"W: {self.w}\nX: {self.x}\nY: {self.y}\nZ: {self.z}"
