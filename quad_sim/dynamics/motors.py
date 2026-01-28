import numpy as np
from numpy import cos as c
from numpy import sin as s
from numpy import tan as t

from quad_sim.math.references.bodyFixed import BodyFixed


class PropMotor:
    def __init__(
        self,
        pos: BodyFixed,
        dir: int,
        k_f: float = 3.13e-5,
        k_m: float = 7.5e-7,
        ang_range: tuple[int | float, int | float] = (0, 600),
    ) -> None:
        if not isinstance(k_f, float):
            raise TypeError("The thrust coefficient must be a float")
        elif k_f <= 0:
            raise ValueError("The thrust coefficient must be > 0")

        if not isinstance(k_m, float):
            raise TypeError("The drag coefficient must be a float")
        elif k_m <= 0:
            raise ValueError("The drag coefficient must be > 0")

        if min(ang_range) < 0 or max(ang_range) <= 0:
            raise ValueError("The ang_range must be positive")

        # Constant Parameters

        self._k_f = k_f
        self._k_m = k_m
        self._min_angVel, self._max_angVel = ang_range  # pyright: ignore[reportAssignmentType]
        self.__rotationDir = BodyFixed.from_Array(np.array([[0], [0], [1]]) * dir)

        # m
        self.position = pos

    # Computes the thrust and the moment about the hinge
    def compute_forces(self, omega: float) -> tuple[BodyFixed, BodyFixed]:
        # T = k_f * w^2
        omega_vec = omega * np.array([[0], [0], [1]])
        thrust_scalar = self.k_f * (omega_vec.T @ omega_vec)[0, 0]

        # thrust_scalar = self.k_f * omega**2
        thrust_vector = np.array([[0], [0], [-thrust_scalar]])
        thrust = BodyFixed.from_Array(thrust_vector, "Motor")

        # M = k_m * w^2
        # Right hand curl
        torque = BodyFixed.from_Array(
            self.k_m * np.linalg.norm(omega_vec) * omega_vec, flag="Motor"
        )

        return thrust, torque

    # TODO: Include function that calculates the velocity of 4-prop tips
    def compute_prop_tip(self):
        return NotImplementedError

    def __str__(self) -> str:
        return f"{self.k_f}, {self.k_m}, {self.min_angVel}, {self.max_angVel}"

    @property
    def k_f(self):
        return self._k_f

    @property
    def k_m(self):
        return self._k_m

    @property
    def min_angVel(self):
        return self._min_angVel

    @property
    def max_angVel(self):
        return self._max_angVel

    @property
    def rotationDir(self):
        return self.__rotationDir

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value: BodyFixed):
        if not isinstance(value, BodyFixed):
            raise TypeError("position must be a BodyFixed")
        elif value.flag != "position":
            raise ValueError("The flag of the position must be position")
        else:
            self._position = value
