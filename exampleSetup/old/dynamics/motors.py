from typing import Type

import numpy as np
from numpy import cos as c
from numpy import sin as s
from numpy import tan as t

from exampleSetup.default.config import MotorConfig
from quad_sim.math.references.bodyFixed import BodyFixed


class PropMotor:
    def __init__(
        self,
        pos: BodyFixed,
        dir: int,  # From -z (1 = CW / -1 = CCW)
        config: MotorConfig,
    ) -> None:
        # Constant Parameters

        self._k_f = config.k_f
        self._k_m = config.k_m
        self._min_angVel, self._max_angVel = config.ang_range
        # ASSUMPTION: dir 1 = +z // -1 = -z
        self.dir = dir
        self.__rotationDir = BodyFixed.from_Array(np.array([[0], [0], [1]]) * dir)

        # m
        self.position = pos

        # Omega squared scalar
        self.throttle = 0

        # ASSUMPTION: 4 wing tips
        self._wing_tip_pos = self.__gen_wing_tips()

    # Computes the thrust and the moment about the hinge
    def compute_forces(self) -> tuple[BodyFixed, BodyFixed]:
        # Stored as the omega squared
        omega = self.throttle
        # T = k_f * w^2
        thrust_scalar = self.k_f * omega

        # thrust_scalar = self.k_f * omega**2
        thrust_vector = np.array([[0], [0], [-thrust_scalar]])
        thrust = BodyFixed.from_Array(thrust_vector, "force")

        # M = k_m * w^2
        # Right hand curl
        torque = -self.k_m * omega * self.rotationDir
        torque.setFlag = "moment"

        return thrust, torque

    def __gen_wing_tips(self, radius=0.10):
        tips = {
            "Tip_{i}": BodyFixed(
                X=radius * np.cos(np.pi / 2 * i),
                Y=radius * np.sin(np.pi / 2 * i),
                Z=0,
                flag="position",
            )
            for i in range(4)
        }

        return tips

    def compute_prop_tip(self) -> dict[str, BodyFixed]:
        vel = {}
        for tip, pos in self._wing_tip_pos.items():
            vel[tip] = (self.rotationDir * self.throttle) * pos

        return vel

    def __str__(self) -> str:
        return f"Kf: {self.k_f}, Km: {self.k_m}, minAng: {self.min_angVel}, maxAnd: {self.max_angVel}"

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

    def getAngSquaredRanges(self):
        return (self.min_angVel, self.max_angVel)

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

    @property
    def wing_tip_pos(self):
        return self._wing_tip_pos

    @wing_tip_pos.setter
    def update_wing_tip_pos(self, value: dict[str, BodyFixed]):
        if (
            not isinstance(value, dict)
            or not all(isinstance(k, str) for k in value)
            or not all(isinstance(v, BodyFixed) for v in value.keys())
        ):
            raise TypeError("Updated wing tip positions must be a dict[str,BodyFixed]")
        else:
            self._wing_tip_pos = value

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value: int | float):
        if not isinstance(value, (int, float)):
            raise TypeError("Throttle must be numerical (int|float)")

        # elif value < self.min_angVel or value > self.max_angVel:
        # if not self.clip_throttle:
        # raise ValueError(
        # f"Throttle cannot exceed the range {self.min_angVel},{self.max_angVel}"
        # )

        # else:
        # value = np.clip(value, self.min_angVel, self.max_angVel)

        self._throttle = value
