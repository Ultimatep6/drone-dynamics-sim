from __future__ import annotations

from dataclasses import astuple
from typing import List, Tuple

import numpy as np

from quad_sim.config import LayoutConfig, MotorConfig, VehicleConfig
from quad_sim.dynamics.motors import PropMotor
from quad_sim.dynamics.rigid_body import RigidBody
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.misc.decorators import subsystem


class Quadcopter(RigidBody):
    def __init__(self, vehConfig: VehicleConfig, drone: "Drone"):
        # Inherit from the RigidBody Class
        super().__init__(*astuple(vehConfig.rigid_config))

        self.drone = drone

        # Overall config
        self.config = vehConfig

        # Physical settings
        self.phyConfig = vehConfig.physical_limits

        # Configuration settings
        self.buildModel(vehConfig.layout_config)

        # Update the physical limits of the quadcopter after motor init
        # self.max_forces()

    def __get_motor_positions(
        self, rotor_configs: List[MotorConfig]
    ) -> list[PropMotor]:
        """
        Calculates the position of all the rotors assuming they are equidistant on the unit circle.
        Returns:
            - list[propMotor]
        """
        dirs = [1, -1]

        # ASSUMPTION: The rotors go CCW from the North basis vector
        # The rotation directions alternate from +z,-z as you go NWSE
        pos = []

        nMotors = self.__rotors
        armLength = self.__arm_length

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
            pos.append(
                PropMotor(
                    BodyFixed.from_Array(disp, flag="position"),
                    dir=dir,
                    config=rotor_configs[i],
                )
            )

        return pos

    def max_forces(self, state: "StateVector") -> tuple[BodyFixed, float, float]:
        """
        Function updates the physical limits of the copter
        """

        # Initiate the total thrust and yaw boundaries
        thrustT = BodyFixed(0, 0, 0, flag="force")
        maxYaw, minYaw = 0, 0

        # Iterate through each motor
        for i, motor in enumerate(self.motorPositions):
            # Set motor throttle to maximum
            motor.throttle = motor.max_angVel
            # Get generated forces and moments
            thrust, torque = motor.compute_forces()
            # Append thrust to the total thrust
            thrustT += thrust
            # Even Motor idx == CW rotating motor -> CCW moment (-z yaw)
            if i % 2 == 0:
                minYaw += torque.vec[2, 0]
            # Odd Motor idx == CCW rotating motor -> CW moment (+z yaw)
            else:
                maxYaw += torque.vec[2, 0]

            # Set motor thrust to minimum
            motor.throttle = motor.min_angVel
        # Update the maximum thrust (-z BF) in the config of the quadcopter
        self.phyConfig.thrust_limits.max_thrust = float(
            EarthFixed.from_BodyFixed(thrustT, state.quaternion).vec[2:3, 0][0]
        )
        # Update the max yaw of the copter
        self.phyConfig.attitude_limits.max_yaw = maxYaw

        return thrustT, maxYaw, minYaw

    def print_layout(self):
        rot_str = "\n".join(
            [
                f"Motor_{rotor} :\n\t\
- Position: {self.motorPositions[rotor].position.vec.T}\n\t\
- Throttle: {self.motorPositions[rotor].throttle}\n\t\
- SpinVec: {self.motorPositions[rotor].rotationDir.vec.T}\n\t\
- Thrust: {self.motorPositions[rotor].compute_forces()[0].vec.T}\n\t\
- Torque: {self.motorPositions[rotor].compute_forces()[1].vec.T}"
                for rotor in range(len(self.motorPositions))
            ]
        )

        print(
            "Drone Layout: \n\
-------------------------\n"
            + rot_str
            + "\n\n------------------------\n\n"
        )

    @property
    def motorPositions(self) -> list[PropMotor]:
        return self._motorPositions

    def buildModel(self, value: LayoutConfig):
        if not isinstance(value, LayoutConfig):
            raise ValueError("Building a the model requires a LayoutConfig")
        else:
            self.__rotors, self.__arm_length = len(value.rotors), value.arm_length

            self._motorPositions = self.__get_motor_positions(value.rotors)

    @property
    def armLength(self):
        return self.__arm_length

    @property
    def rotors(self):
        return self.__rotors
