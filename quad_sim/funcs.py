from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from quad_sim.references.bodyFixed import BodyFixed
    from quad_sim.references.earthFixed import EarthFixed
    from quad_sim.bases.state import StateVector
    from quad_sim.orientation.quaternion import Quaternion

"""
The rotation convention is ZYX in Eurlerian or yaw-pitch-roll
The rotation transform is w-XYZ in Quaternion or roll-pitch-yaw
"""



def _position_of_point(
        r: BodyFixed,
        state: StateVector,
        extras: BodyFixed | None = None,
    ) -> EarthFixed:
        """
        Calculate the position of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        from quad_sim.references.bodyFixed import BodyFixed as _BodyFixed
        from quad_sim.references.earthFixed import EarthFixed as _EarthFixed

        if extras is None:
            extras = _BodyFixed(0, 0, 0)
        return state.position + _EarthFixed.from_BodyFixed(
            r + extras, state.quaternion, flag="position"
        )
    
def _velocity_of_point(
        state: StateVector,
        r: BodyFixed,
        extras: BodyFixed | None = None,
    ) -> BodyFixed:
        """
        Calculate the velocity of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        from quad_sim.references.bodyFixed import BodyFixed as _BodyFixed

        if extras is None:
            extras = _BodyFixed(0, 0, 0, flag="velocity")
        return state.velocity + (state.omega * r).changeFlag("velocity") + extras

def _acceleration_of_point(
        state: StateVector,
        r: BodyFixed,
        extras: BodyFixed | None = None,
    ) -> BodyFixed:
        """
        Calculate the acceleration of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        from quad_sim.references.bodyFixed import BodyFixed as _BodyFixed

        if extras is None:
            extras = _BodyFixed(0, 0, 0, flag="acceleration")
        a = (state.alpha * r).changeFlag("acceleration")
        b = (state.alpha * (state.alpha * r)).changeFlag("acceleration")

        return state.acceleration - a + b

def asin(x):
    return np.arcsin(np.clip(x, -1.0, 1.0))


def acos(x):
    return np.arccos(np.clip(x, -1.0, 1.0))


def atan2(y, x):
    return np.arctan2(y, x)

def _get_body_to_inertial(quaternion_orientation: np.ndarray) -> np.ndarray:
    """
    Returns the rotation matrix from body (NED) to inertial (NWU) plane.
    NWU: X-North, Y-West, Z-Up. Gravity will be [0, 0, -g].
    """
    # ... (Keep your existing validation logic) ...

    e0, e1, e2, e3 = tuple(quaternion_orientation[:, 0])

    # Standard Body -> NED Rotation Matrix
    R_ned = np.array(
        [
            [
                e1**2 + e0**2 - e2**2 - e3**2,
                2 * (e1 * e2 - e3 * e0),
                2 * (e1 * e3 + e2 * e0),
            ],
            [
                2 * (e1 * e2 + e3 * e0),
                e2**2 + e0**2 - e1**2 - e3**2,
                2 * (e2 * e3 - e1 * e0),
            ],
            [
                2 * (e1 * e3 - e2 * e0),
                2 * (e2 * e3 + e1 * e0),
                e3**2 + e0**2 - e1**2 - e2**2,
            ],
        ]
    )

    # Convert NED result to NWU
    # Row 0 (North) remains same
    # Row 1 (East -> West) negated
    # Row 2 (Down -> Up) negated
    # T_ned_to_nwu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    T_ned_to_nwu = np.identity(3)

    return T_ned_to_nwu @ R_ned


def _get_inertial_to_body(quaternion_orientation: np.ndarray) -> np.ndarray:
    return _get_body_to_inertial(quaternion_orientation).T


def compute_aB(
    mass: int | float, F_B: BodyFixed, omega_B: BodyFixed, vel_B: BodyFixed
) -> BodyFixed:
    # --- mass checks ---
    if not isinstance(mass, (int, float)):
        raise TypeError("mass must be an int or float")

    if mass <= 0:
        raise ValueError("mass must be positive")

    # --- F_B checks ---
    if not isinstance(F_B, BodyFixed):
        raise TypeError("F_B must be a BodyFixed vector")

    if F_B.vec.shape != (3, 1):
        raise ValueError("F_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(F_B.vec)):
        raise ValueError("F_B contains non-finite values")

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- vel_B checks ---
    if not isinstance(vel_B, BodyFixed):
        raise TypeError("vel_B must be a BodyFixed vector")

    if vel_B.vec.shape != (3, 1):
        raise ValueError("vel_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(vel_B.vec)):
        raise ValueError("vel_B contains non-finite values")

    # --- compute acceleration ---
    res = F_B * (1.0 / mass) - omega_B * vel_B
    res.flag = "acceleration"
    return res


def compute_alphaB(
    inertia: np.ndarray, M_B: BodyFixed, omega_B: BodyFixed
) -> BodyFixed:
    # --- inertia checks ---
    if not isinstance(inertia, np.ndarray):
        raise TypeError("inertia must be a numpy ndarray")

    if inertia.shape != (3, 3):
        raise ValueError("inertia must be a 3×3 matrix")

    if not np.all(np.isfinite(inertia)):
        raise ValueError("inertia contains non-finite values")

    if np.linalg.det(inertia) == 0:
        raise ValueError("inertia matrix must be invertible")

    # --- M_B checks ---
    if not isinstance(M_B, BodyFixed):
        raise TypeError("M_B must be a BodyFixed vector")

    if M_B.vec.shape != (3, 1):
        raise ValueError("M_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(M_B.vec)):
        raise ValueError("M_B contains non-finite values")

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- compute angular acceleration ---
    rhs = M_B.vec - np.cross(omega_B.vec, inertia @ omega_B.vec, axis=0)
    alpha_vec = np.linalg.solve(inertia, rhs)

    return BodyFixed.from_Array(alpha_vec, flag="ang_acceleration")


def compute_q_rate(quaternion: Quaternion, omega_B: BodyFixed) -> Quaternion:
    # --- quaternion checks ---
    quaternion = quaternion.normalized()

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- compute quaternion rate ---
    p, q, r = omega_B.vec.T.ravel()

    mat = np.array(
        [
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0],
        ],
        dtype=float,
    )

    return Quaternion.unpackArray( 0.5 * (mat @ quaternion.as_np()) )