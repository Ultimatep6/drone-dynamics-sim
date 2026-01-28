from typing import Tuple

import numpy as np

from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.math.rotations import _quaternion_to_eulerian, norm_quaternion


class RigidBody:
    def __init__(self, mass, Ixx, Iyy, Izz) -> None:
        # Configuration variables
        self.mass = mass
        # ASSUMPTION: The inertia of the object is assumed diagonal, the drone is symmetric on all axis
        self.inertia_tensor = np.diag([Ixx, Iyy, Izz])

        # State Vector
        self.position = EarthFixed.from_Array(np.zeros((3, 1)))
        self.velocity = BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity")
        self.quaternion = np.array([[1], [0], [0], [0]])
        self.omega = BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity")

        # Misc
        self.acceleration = EarthFixed.from_Array(np.zeros((3, 1)), flag="acceleration")
        self.alpha = BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_acceleration")

    # TODO: Implement extras as the extra component for the prop tip

    def _position_of_point(self, r: BodyFixed, extras=None) -> EarthFixed:
        return self.position + EarthFixed.from_BodyFixed(
            r, self.quaternion, flag="position"
        )

    def _velocity_of_point(self, r: BodyFixed, extras=None) -> BodyFixed:
        return self.velocity + BodyFixed.from_Array(
            (self.omega * r).vec, flag="velocity"
        )

    def _acceleration_of_point(self, r: BodyFixed, extras=None) -> EarthFixed:
        return (
            self.acceleration
            - EarthFixed.from_BodyFixed(
                self.alpha * r, self.quaternion, flag="acceleration"
            )
            + EarthFixed.from_BodyFixed(
                self.omega * (self.omega * r), self.quaternion, flag="acceleration"
            )
        )

    def get_stateVec(self):
        return [self.position, self.velocity, self.quaternion, self.omega]

    def update_stateVec(
        self, stateVec: Tuple[EarthFixed, BodyFixed, np.ndarray, BodyFixed]
    ):
        if len(stateVec) != 4:
            raise ValueError("State vector must have 4 components")

        if not isinstance(stateVec[0], EarthFixed):
            raise TypeError("Position in state vector must be EarthFixed")

        if not isinstance(stateVec[1], BodyFixed):
            raise TypeError("Velocity in state vector must be BodyFixed")

        if isinstance(stateVec[2], np.ndarray):
            if stateVec[2].shape != (4, 1):
                raise TypeError("Quaternion in state vector must be of shape (4,1)")

            if np.linalg.norm(stateVec[2]) != 1:
                raise TypeError("Quaternion in state vector must be normalized")
        else:
            raise TypeError("Quaternion in state vector must be a np.ndarray")

        if not isinstance(stateVec[3], BodyFixed):
            raise TypeError("Angular Velocity in state vector must be BodyFixed")

        self.position, self.velocity, self.quaternion, self.omega = stateVec

    def __str__(self) -> str:
        pos_str = f"\tX: {self.position.vec[0, 0]}\n\tY: {self.position.vec[1, 0]}\n\tZ: {self.position.vec[2, 0]}"
        vel_str = f"\tX: {self.velocity.vec[0, 0]}\n\tY: {self.velocity.vec[1, 0]}\n\tZ: {self.velocity.vec[2, 0]}"
        eul_angle = np.degrees(_quaternion_to_eulerian(self.quaternion))
        eul_str = f"\tRoll: {eul_angle[0, 0]}\n\tPitch: {eul_angle[1, 0]}\n\tYaw: {eul_angle[2, 0]}"
        ang_vel_str = f"\tX: {self.omega.vec[0, 0]}\n\tY: {self.omega.vec[1, 0]}\n\tZ: {self.omega.vec[2, 0]}"

        return (
            "Inertial Position:\n"
            + pos_str
            + "\n\n------------------------\n\n"
            + "Velocity (BF):\n"
            + vel_str
            + "\n\n------------------------\n\n"
            + "Orientation:\n"
            + eul_str
            + "\n\n------------------------\n\n"
            + "Angular Velocity (BF):\n"
            + ang_vel_str
            + "\n------------------------"
            + "\n------------------------\n\n"
        )

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, value: int | float):
        if not isinstance(value, (int, float)):
            raise TypeError("mass must be a int|float")
        else:
            self._mass = value

    @property
    def inertia_tensor(self):
        return self._inertia_tensor

    @inertia_tensor.setter
    def inertia_tensor(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError("inertia_tensor must be a np.ndarray")

        if value.shape != (3, 3):
            raise ValueError("inertia_tensor must be a 3x3 matrix")

        if not np.issubdtype(value.dtype, np.number):
            raise TypeError("inertia_tensor must contain numeric values")

        # Check diagonal
        if not np.allclose(value, np.diag(np.diag(value)), atol=1e-12):
            raise ValueError("inertia_tensor must be diagonal")

        self._inertia_tensor = value

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value: EarthFixed):
        if not isinstance(value, EarthFixed):
            raise TypeError("position must be a EarthFixed")

        elif not np.issubdtype(value.vec.dtype, np.number):
            raise TypeError("position must be a numerical vector")

        else:
            self._position = value

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value: BodyFixed):
        if not isinstance(value, BodyFixed):
            raise TypeError("velocity must be a BodyFixed")
        elif not np.issubdtype(value.vec.dtype, np.number):
            raise TypeError("velocity must be a numerical vector")
        else:
            self._velocity = value

    @property
    def acceleration(self):
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value: EarthFixed):
        if not isinstance(value, EarthFixed):
            raise TypeError("acceleration must be a EarthFixed")
        elif not np.issubdtype(value.vec.dtype, np.number):
            raise TypeError("acceleration must be a numerical vector")
        else:
            self._acceleration = value

    @property
    def quaternion(self):
        return self._quaternion

    @quaternion.setter
    def quaternion(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError("quaternion must be a np.ndarray")
        elif not np.issubdtype(value.dtype, np.number):
            raise TypeError("quaternion must be a numerical vector")
        elif value.shape != (4, 1):
            raise TypeError("quaternion must be a 4x1 vector")
        elif np.linalg.norm(value) != 1:
            raise ValueError("quaternion must be a unit 4x1 vector")

        else:
            self._quaternion = value
