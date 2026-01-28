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


def norm_quaternion(quaternion_orientation: np.ndarray):
    """
    Normalizes the quaternion
    """
    if not isinstance(quaternion_orientation, np.ndarray):
        raise TypeError("quaternion_orientation must be np.ndarray")

    elif quaternion_orientation.shape != (4, 1):
        raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

    return quaternion_orientation / np.linalg.norm(quaternion_orientation)


def inverse_quaternion(quaternion_orientation: np.ndarray):
    """
    Generates an opposite rotation by doing inverse
    """
    if not isinstance(quaternion_orientation, np.ndarray):
        raise TypeError("quaternion_orientation must be np.ndarray")

    elif quaternion_orientation.shape != (4, 1):
        raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

    elif np.linalg.norm(quaternion_orientation) != 1:
        raise ValueError("quaternion_orientation is not unit normalized")

    quaternion_orientation[1:] *= -1
    return quaternion_orientation


def _eulerian_to_quaternion(eulerian_orientation: np.ndarray):
    """
    This function converts an eulerian angle to a quaternion angle
    """
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
    """
    This function converts a quaternion angle to an eulerian angle
    """
    if not isinstance(quaternion_orientation, np.ndarray):
        raise TypeError("quaternion_orientation must be np.ndarray")

    elif quaternion_orientation.shape != (4, 1):
        raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

    elif np.linalg.norm(quaternion_orientation) != 1:
        raise ValueError("quaternion_orientation is not unit normalized")

    e0, e1, e2, e3 = tuple(quaternion_orientation[:, 0])

    eulerian = np.array(
        [
            [At((2 * (e0 * e1 + e2 * e3)), (e0**2 + e3**2 - e1**2 - e2**2))],
            [As(2 * (e0 * e2 - e1 * e3))],
            [At((2 * (e0 * e3 + e1 * e2)), (e0**2 + e1**2 - e2**2 - e3**2))],
        ]
    )

    return eulerian


def _quaternion_to_eulerian_VEL(quaternion_rate: np.ndarray) -> np.ndarray:
    """
    This function converts a quaternion angle to an eulerian angle
    """
    if not isinstance(quaternion_rate, np.ndarray):
        raise TypeError("quaternion_rate must be np.ndarray")

    elif quaternion_rate.shape != (4, 1):
        raise TypeError("quaternion_rate must be np.ndarray of shape (4,1)")

    e0, e1, e2, e3 = tuple(quaternion_rate[:, 0])

    print(2 * (e0 * e2 - e1 * e3))

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

    elif np.linalg.norm(quaternion_orientation) != 1:
        raise ValueError("quaternion_orientation is not unit normalized")

    e0, e1, e2, e3 = tuple(quaternion_orientation[:, 0])

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


def quaternion_rotation(
    quaternion_orientation: np.ndarray,
    quaternion_transformation: np.ndarray,
    debug: bool = True,
):
    if debug:
        if not isinstance(quaternion_orientation, np.ndarray):
            raise TypeError("quaternion_orientation must be np.ndarray")

        elif quaternion_orientation.shape != (4, 1):
            raise TypeError("quaternion_orientation must be np.ndarray of shape (4,1)")

        elif np.linalg.norm(quaternion_orientation) != 1:
            raise ValueError("quaternion_orientation is not unit normalized")

        if not isinstance(quaternion_transformation, np.ndarray):
            raise TypeError("quaternion_transformation must be np.ndarray")

        elif quaternion_transformation.shape != (4, 1):
            raise TypeError(
                "quaternion_transformation must be np.ndarray of shape (4,1)"
            )

        elif np.linalg.norm(quaternion_transformation) != 1:
            raise ValueError("quaternion_transformation is not unit normalized")

    qS, eS = quaternion_orientation[0:1, 0:], quaternion_transformation[0:1, 0:]
    qV, eV = quaternion_orientation[1:, 0:], quaternion_transformation[1:, 0:]

    scal = qS * eS - np.dot(qV.T, eV)
    vec = qS * eV + eS * qV + np.cross(qV, eV, axis=0)

    q = np.zeros((4, 1))
    q[0, :] = scal
    q[1:, :] = vec

    return q
