from ast import Not

import numpy as np

from quad_sim.dynamics.motors import PropMotor
from quad_sim.dynamics.rigid_body import RigidBody
from quad_sim.math.references.bodyFixed import BodyFixed


class Quadcopter(RigidBody):
    def __init__(
        self,
        mass: int,
        Ixx: float,
        Iyy: float,
        Izz: float,
        rotors: int = 4,
        arm_length: float = 0.30,
    ) -> None:
        # Inherit from the RigidBody Class
        super().__init__(mass, Ixx, Iyy, Izz)

        # Configuration settings
        self.motorPositions = (rotors, arm_length)

    def __get_motor_positions(
        self, nMotors: int, armLength: int | float
    ) -> list[PropMotor]:
        """
        Calculates the position of all the rotors assuming they are equidistant on the unit circle.
        The returned thing is a list of PropMotor class objects
        """
        dirs = [1, -1]

        # ASSUMPTION: The rotors go CCW from the North basis vector
        pos = []

        for i, rotor in enumerate(range(nMotors)):
            disp = np.array(
                [
                    [armLength * np.cos((2 * np.pi / nMotors) * rotor)],
                    [armLength * np.sin((2 * np.pi / nMotors) * rotor)],
                    [0],
                ]
            )
            disp[np.abs(disp) < 1e-15] = 0
            dir = dirs[i % 2]
            pos.append(PropMotor(BodyFixed.from_Array(disp, flag="position"), dir=dir))

        return pos

    # TODO: Implement total forces calculations
    def _get_total_forces():
        raise NotImplementedError

    def print_layout(self) -> str:
        rot_str = "\n".join(
            [
                f"Motor_{rotor} : {self.motorPositions[rotor].position.vec.T}"
                for rotor in range(len(self.motorPositions))
            ]
        )

        return (
            "Drone Layout: \n\
-------------------------\n"
            + rot_str
            + "\n\n------------------------\n\n"
        )

    @property
    def motorPositions(self):
        return self._motorPositions

    @motorPositions.setter
    def motorPositions(self, value: tuple[int, int | float]):
        if len(value) != 2:
            raise ValueError(
                "The arguments for setting motorPositions must be of len 2"
            )
        else:
            nMotor, armLength = value
            if not isinstance(nMotor, (int, float)):
                raise TypeError("motors must be an int or float")
            if not isinstance(armLength, (int, float)):
                raise TypeError("armLength must be an int or float")

            if nMotor < 2:
                raise ValueError("rotors must be at least 2 ")
            if armLength <= 0:
                raise ValueError("armlength must be positive non-zero meters")

            self._motorPositions = self.__get_motor_positions(nMotor, armLength)
