from typing import Tuple

import numpy as np

from quad_sim.dynamics.state import StateVector
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed


class RigidBody:
    def __init__(self, mass, Ixx, Iyy, Izz) -> None:
        # Configuration variables
        self.mass = mass
        # ASSUMPTION: The inertia of the object is assumed diagonal, the drone is symmetric on all axis
        self.inertia_tensor = np.diag([Ixx, Iyy, Izz])

    def _position_of_point(
        self,
        r: BodyFixed,
        state: StateVector,
        extras: BodyFixed = BodyFixed(0, 0, 0),
    ) -> EarthFixed:
        """
        Calculates the position of any point on the rigid body in the inertial frame
        """
        return state.position + EarthFixed.from_BodyFixed(
            r + extras, state.quaternion, flag="position"
        )

    def _velocity_of_point(
        self,
        state: StateVector,
        r: BodyFixed,
        extras: BodyFixed = BodyFixed(0, 0, 0, flag="velocity"),
    ) -> BodyFixed:
        return state.velocity + (state.omega * r).changeFlag("velocity") + extras

    def _acceleration_of_point(
        self,
        state: StateVector,
        r: BodyFixed,
        extras=BodyFixed(0, 0, 0, flag="acceleration"),
    ) -> BodyFixed:
        a = (state.alpha * r).changeFlag("acceleration")
        b = (state.alpha * (state.alpha * r)).changeFlag("acceleration")

        return state.acceleration - a + b

    # def update_stateVec(
    # self, stateVec: Tuple[EarthFixed, BodyFixed, np.ndarray, BodyFixed]
    # ):
    # if len(stateVec) != 4:
    # raise ValueError("State vector must have 4 components")

    # if not isinstance(stateVec[0], EarthFixed):
    # raise TypeError("Position in state vector must be EarthFixed")

    # if not isinstance(stateVec[1], BodyFixed):
    # raise TypeError("Velocity in state vector must be BodyFixed")

    # if isinstance(stateVec[2], np.ndarray):
    # if stateVec[2].shape != (4, 1):
    # raise TypeError("Quaternion in state vector must be of shape (4,1)")

    # if not np.isclose(np.linalg.norm(stateVec[2]), 1.0, atol=1e-12):
    # raise TypeError("Quaternion in state vector must be normalized")
    # else:
    # raise TypeError("Quaternion in state vector must be a np.ndarray")

    # if not isinstance(stateVec[3], BodyFixed):
    # raise TypeError("Angular Velocity in state vector must be BodyFixed")

    # self.position, self.velocity, self.quaternion, self.omega = stateVec

    def __str__(self) -> str:
        return f"Mass : {self.mass}kg\nInertia Ixx: {self.inertia_tensor[0, 0]}\nInertia Iyy: {self.inertia_tensor[1, 1]}\nInertia Izz: {self.inertia_tensor[2, 2]}"

        # vel_str = f"\tX: {self.velocity.vepos_str = f"\tX: {self.position.vec[0, 0]}\n\tY: {self.position.vec[1, 0]}\n\tZ: {self.position.vec[2, 0]}"
        # vel_str = f"\tX: {self.velocity.vevel_str = f"\tX: {self.velocity.vec[0, 0]}\n\tY: {self.velocity.vec[1, 0]}\n\tZ: {self.velocity.vec[2, 0]}"
        # vel_str = f"\tX: {self.velocity.veeul_angle = np.degrees(_quaternion_to_eulerian(self.quaternion))
        # vel_str = f"\tX: {self.velocity.veeul_str = f"\tRoll: {eul_angle[0, 0]}\n\tPitch: {eul_angle[1, 0]}\n\tYaw: {eul_angle[2, 0]}"
        # vel_str = f"\tX: {self.velocity.veang_vel_str = f"\tX: {self.omega.vec[0, 0]}\n\tY: {self.omega.vec[1, 0]}\n\tZ: {self.omega.vec[2, 0]}"

        # vel_str = f"\tX: {self.velocity.vereturn (
        # vel_str = f"\tX: {self.velocity.ve"Inertial Position:\n"
        # vel_str = f"\tX: {self.velocity.ve+ pos_str
        # vel_str = f"\tX: {self.velocity.ve+ "\n\n------------------------\n\n"
        # vel_str = f"\tX: {self.velocity.ve+ "Velocity (BF):\n"
        # vel_str = f"\tX: {self.velocity.ve+ vel_str
        # vel_str = f"\tX: {self.velocity.ve+ "\n\n------------------------\n\n"
        # vel_str = f"\tX: {self.velocity.ve+ "Orientation:\n"
        # vel_str = f"\tX: {self.velocity.ve+ eul_str
        # vel_str = f"\tX: {self.velocity.ve+ "\n\n------------------------\n\n"
        # vel_str = f"\tX: {self.velocity.ve+ "Angular Velocity (BF):\n"
        # vel_str = f"\tX: {self.velocity.ve+ ang_vel_str
        # vel_str = f"\tX: {self.velocity.ve+ "\n------------------------"
        # vel_str = f"\tX: {self.velocity.ve+ "\n------------------------\n\n"
        # vel_str = f"\tX: {self.velocity.ve)

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
