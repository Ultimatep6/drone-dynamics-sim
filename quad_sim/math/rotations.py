import numpy as np
from numpy import asin as As
from numpy import atan2 as At
from numpy import cos as c
from numpy import sin as s


def _eulerian_to_quaternion(eulerian_orientation: np.ndarray):
    if not isinstance(eulerian_orientation, np.ndarray):
        raise TypeError("eulerian_orientation must be np.ndarray")
    elif eulerian_orientation.shape != (3, 1):
        raise TypeError("eulerian_orientation must be np.ndarray of shape (3,1)")

    pitch, roll, yaw = tuple(eulerian_orientation[:, 1])

    quaternion = np.array(
        [
            [
                c(yaw / 2) * c(pitch / 2) * c(roll / 2)
                + s(yaw / 2) * s(pitch / 2) * s(roll / 2)
            ],
            [
                c(yaw / 2) * c(pitch / 2) * s(roll / 2)
                - s(yaw / 2) * s(pitch / 2) * c(roll / 2)
            ],
            [
                c(yaw / 2) * s(pitch / 2) * c(roll / 2)
                + s(yaw / 2) * c(pitch / 2) * s(roll / 2)
            ],
            [
                s(yaw / 2) * c(pitch / 2) * c(roll / 2)
                - c(yaw / 2) * s(pitch / 2) * s(roll / 2)
            ],
        ]
    )

    return quaternion


def _quaternion_to_eulerian(quaternion_orientation: np.ndarray) -> np.ndarray:
    if not isinstance(quaternion_orientation, np.ndarray):
        raise TypeError("quaternion_orientation must be np.ndarray")

    elif quaternion_orientation.shape != (4, 1):
        raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

    elif np.abs(quaternion_orientation) != 1:
        raise ValueError("quaternion_orientation is not unit normalized")

    e0, e1, e2, e3 = tuple(quaternion_orientation[:, 1])

    eulerian = np.array(
        [
            [At((2 * (e0 * e1 + e2 * e3)), (e0**2 + e3**2 - e1**2 - e2**2))],
            [As(2 * (e0 * e2 - e1 * e3))],
            [At((2 * (e0 * e3 + e1 * e2)), (e0**2 + e1**2 - e2**2 - e3**2))],
        ]
    )

    return eulerian


def _get_body_to_inertial(quaternion_orientation: np.ndarray) -> np.ndarray:
    """
    Returns a the rotation matrix from body to inertial (Earth) plane.
    """
    if not isinstance(quaternion_orientation, np.ndarray):
        raise TypeError("quaternion_orientation must be np.ndarray")

    elif quaternion_orientation.shape != (4, 1):
        raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

    elif np.abs(quaternion_orientation) != 1:
        raise ValueError("quaternion_orientation is not unit normalized")

    e0, e1, e2, e3 = tuple(quaternion_orientation[:, 1])

    rotation_matrix = np.array(
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

    return rotation_matrix


def _get_inertial_to_body(quaternion_orientation: np.ndarray) -> np.ndarray:
    return _get_body_to_inertial(quaternion_orientation).T
