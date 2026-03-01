import numpy as np
from numpy import cos as c
from numpy import sin as s

from quad_sim.math.safe_trig import acos as Ac
from quad_sim.math.safe_trig import asin as As
from quad_sim.math.safe_trig import atan2 as At

"""
The rotation convention is ZYX in Eurlerian or yaw-pitch-roll
The rotation transform is w-XYZ in Quaternion or roll-pitch-yaw
"""


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
