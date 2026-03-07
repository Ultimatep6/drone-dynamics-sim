from __future__ import annotations

from typing import List, Tuple

import numpy as np

from exampleSetup.default.config import IntegratorConfig, VehicleConfig
from quad_sim.control.config.autopilotConf import AutopilotConfig
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.control.RC import RC
from quad_sim.control.systems.Allocator import Allocator
from quad_sim.control.systems.AutoPilot import AutoPilot
from quad_sim.dynamics.quadcopter import Quadcopter
from exampleSetup.default.state import StateVector
from quad_sim.math.integrators import ForwardEulerIntegrator
from quad_sim.math.quaternion import Quaternion
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed

INTEGRATOR_REGISTRY = {
    "ForwardEuler": ForwardEulerIntegrator,
    # "RK2": RK2Integrator,
}


class Drone:
    def __init__(self, config: VehicleConfig):
        self.id = config.vehicleID
        self.limits = config.physical_limits

        self.model = Quadcopter(config, self)  # Quadcopter (plant)

        self.state = (
            config.init_state if config.init_state is not None else StateVector()
        )
        # self.model.max_forces(self.state)

        self.controller = AutoPilot(self)
        self.pids = self.setupPids(config.pid_config)  # owns PID instances
        self.integrator = self.setupIntegrators(config.integrator)
        self.allocator = Allocator(self)

        self.RC = RC()

    # TODO: Setup basic GravityModel
    def step(self, envContext: EnvironmentSettings | None = None):
        # calculate the desired inputs
        desired = self.controller.rc_to_command(self.RC)

        # calulate the required
        req = self.controller.stepPID(desired)

        # set the throttles
        self.setThrottle(self.allocator.calcRates(req), "all")

        # Integrate and update the state
        self.integrator.step_forward_euler()

    def updateStateVector(
        self,
        vec: StateVector
        | Tuple[EarthFixed, BodyFixed, Quaternion, BodyFixed, BodyFixed, BodyFixed],
    ):
        """
        Updates the state of the drone
        """

        if isinstance(vec, Tuple) and len(vec) == 6:
            self.state.position.vec[:] = vec[0].vec[:]
            self.state.velocity.vec[:] = vec[1].vec[:]
            self.state.quaternion = vec[2]
            self.state.omega.vec[:] = vec[3].vec[:]
            self.state.acceleration.vec[:] = vec[4].vec[:]
            self.state.alpha.vec[:] = vec[5].vec[:]

        elif not isinstance(vec, StateVector):
            raise TypeError("The new stateVector must be a stateVector dataclass")

        else:
            self.state = vec

    def setThrottle(self, throttle: int | float | np.ndarray, motorN: int | str):
        motors = self.model.motorPositions
        n = len(motors)

        # -----------------------------
        # Normalize throttle input
        # -----------------------------
        if isinstance(throttle, (int, float)):
            # Scalar mode
            throttle_scalar = float(throttle)
            throttle_vec = None

        else:
            # Vector mode
            arr = np.asarray(throttle)

            # Accept (N,) or (N,1)
            if arr.ndim == 1:
                throttle_vec = arr
            elif arr.ndim == 2 and arr.shape[1] == 1:
                throttle_vec = arr[:, 0]
            else:
                raise TypeError(
                    f"Throttle array must be shape (N,) or (N,1), got {arr.shape}"
                )

            if len(throttle_vec) != n:
                raise ValueError(
                    f"Throttle vector must have {n} elements, got {len(throttle_vec)}"
                )

            throttle_scalar = None

        # -----------------------------
        # Apply throttle
        # -----------------------------
        if motorN == "all":
            if throttle_vec is not None:
                # Per-motor throttle
                for motor, value in zip(motors, throttle_vec):
                    motor.throttle = float(value)
            else:
                # Scalar throttle to all motors
                for motor in motors:
                    motor.throttle = throttle_scalar
            return

        # -----------------------------
        # Single motor
        # -----------------------------
        if not isinstance(motorN, int):
            raise TypeError(f"motorN must be an int or 'all', got {motorN!r}")

        if not (0 <= motorN < n):
            raise IndexError(f"Motor index {motorN} out of range 0–{n - 1}")

        if throttle_vec is not None:
            raise TypeError("Cannot pass a throttle vector when setting a single motor")

        motors[motorN].throttle = throttle_scalar

    # TODO: Find a way to pass through the environment
    def _get_total_forces(self, gravity: bool = True):
        _, _, quaternion, _ = self.getStateVec()

        # Store the total thrust and moments in the body frame
        thrustT = BodyFixed(0, 0, 0, flag="force")
        torqueT = BodyFixed(0, 0, 0, flag="moment")
        for motor in self.getMotors():
            thrust, torque = motor.compute_forces()
            thrustT += thrust
            torqueT += motor.position * thrust
            torqueT += torque

        # ADD EXTERNAL FORCES
        print(f"Total thrust T BF NO Grav: {thrustT.vec.T}")
        if gravity:
            # Convert gravity to body frame
            G = BodyFixed.from_EarthFixed(
                EarthFixed(0, 0, 9.81, "force"), self.state.quaternion
            )
            print(G.vec)

            thrustT += self.model.mass * G
        print(f"Total thrust T BF + Grav: {thrustT.vec.T}")
        print(f"Total moment M BF: {torqueT.vec.T}")
        return thrustT, torqueT

    def get_subsystems(self):
        subsystems = []
        for attr in vars(self).values():
            if hasattr(attr, "_subsystem_name"):
                subsystems.append(attr)
        return subsystems

    def setupPids(self, config: AutopilotConfig):
        return config.gains.controllers

    def setupIntegrators(self, config: IntegratorConfig):
        return INTEGRATOR_REGISTRY[config.setting.value](config, self)

    def getMotors(self):
        """
        Returns the motorPositions
        """
        return self.model.motorPositions

    def getFlightMode(self):
        """
        Returns the current mode of flight
        """
        # TODO: Shift mode to drone ownership
        return self.controller.mode

    def getStateVec(self):
        """
        Returns the current state vector
        """
        return (
            self.state.position,
            self.state.velocity,
            self.state.quaternion,
            self.state.omega,
        )

    def setMode(self, mode: FlightMode):
        """
        Updates the flight mode
        """
        self.controller.mode = mode

    def updateRC(self, axis, input):
        self.RC.updateInputs(axis, input)
