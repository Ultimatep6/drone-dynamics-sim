from __future__ import annotations

from typing import List

import h5py
import numpy as np

from quad_sim.control.config.allocatorInput import AllocatorInput
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.dynamics.motors import PropMotor
from quad_sim.misc.decorators import subsystem
from quad_sim.utils.logFuncs import create_schema


@subsystem("Allocator")
class Allocator:
    def __init__(self, drone: "Drone", logSchema: dict | None = None) -> None:
        self.drone = drone
        self.mode = drone.getFlightMode()
        motors = drone.getMotors()

        # Make constant matrix
        self.MAT = self.initMAT(motors)
        self.MAT_i = np.linalg.pinv(self.MAT)

        self.motors = motors

        # Stores the ranges of the omegas**2
        self.min_limits = np.array([m.getAngSquaredRanges()[0] for m in motors])
        self.max_limits = np.array([m.getAngSquaredRanges()[1] for m in motors])

        self.logSchema = logSchema
        if self.logSchema is None:
            self.logSchema = self.get_log_definition()
        self._telem = {field: 0.0 for field in self.logSchema.keys()}

    def calcRates(self, input: AllocatorInput | List[float]):
        """
        Calculates the throttles of the motors by taking the Thrust and Moments
        """
        if not isinstance(input, AllocatorInput) and not (
            isinstance(input, list) and len(input) == 4
        ):
            raise TypeError(
                "Allocator input for solving omegas must be an Allocator Input or list of 4"
            )
        elif isinstance(input, list):
            input = AllocatorInput(
                thrust=input[0],
                momentX=input[1],
                momentY=input[2],
                momentZ=input[3],
            )

        w_sq = self.get_raw_omega_sq(input)
        mode = self.drone.getFlightMode()

        self._telem["omega_squared"] = w_sq
        self._telem["flightMode"] = mode

        # print(f"The current mode is {mode}")

        if mode == FlightMode.STABILIZED:
            return self.desaturate_stabilized(w_sq)

        elif mode == FlightMode.ALTITUDE:
            return self.desaturate_altitude(input)

        else:
            raise ValueError("The flight mode has not been added")

    def get_raw_omega_sq(self, input: AllocatorInput):
        """Step 1: Raw linear solver (Source 2 / Matrix Γ inverse)"""
        b = input.as_np()
        return (self.MAT_i @ b).flatten()

    def desaturate_stabilized(self, omega_sq_raw):
        """Priority: Attitude > Thrust"""
        omega_sq = np.copy(omega_sq_raw)

        # 1. Check for floor violations (Under-saturation)
        # Shift all motors up by the largest violation
        diff_min = self.min_limits - omega_sq
        max_floor_violation = np.max(diff_min)
        if max_floor_violation > 0:
            omega_sq += max_floor_violation

        # 2. Check for ceiling violations (Over-saturation)
        # Shift all motors down by the largest violation
        diff_max = omega_sq - self.max_limits
        max_ceil_violation = np.max(diff_max)
        if max_ceil_violation > 0:
            omega_sq -= max_ceil_violation

        # 3. Hard clamp as a fail-safe
        return np.clip(omega_sq, self.min_limits, self.max_limits)

    def desaturate_altitude(self, input: AllocatorInput):
        omega_thrust = (self.MAT_i[:, 0] * input.thrust).flatten()
        omega_moments = (self.MAT_i[:, 1:] @ input.as_np()[1:, :]).flatten()

        # 1. Calculate margins to BOTH limits
        # How much can we increase? (Upper)
        # How much can we decrease? (Lower)
        upper_margin = self.max_limits - omega_thrust
        lower_margin = omega_thrust - self.min_limits

        # The 'available' room for a moment is the MINIMUM of the two margins.
        # This ensures that if we can't go UP, we don't go DOWN (preserving balance).
        allowed_margin = np.minimum(upper_margin, lower_margin)

        # 2. Prevent negative margins (if thrust itself is out of bounds)
        allowed_margin = np.maximum(allowed_margin, 0)

        # 3. Calculate scaling factor
        scale = 1.0
        for i in range(len(self.motors)):
            abs_mom = abs(omega_moments[i])
            if abs_mom > allowed_margin[i]:
                # If we have zero margin, s becomes 0
                s = allowed_margin[i] / abs_mom if abs_mom > 1e-6 else 0
                scale = min(scale, s)

        # 4. Final output
        # With scale=0 (in your test case), this returns only omega_thrust
        print(omega_thrust + (omega_moments * scale))
        return np.clip(
            omega_thrust + (omega_moments * scale), self.min_limits, self.max_limits
        )

    def initMAT(self, motorPositions: list[PropMotor]):
        nMotors = len(motorPositions)
        MAT = np.zeros((4, nMotors))

        for i, motor in enumerate(motorPositions):
            x = motor.position.vec[0, 0]
            y = motor.position.vec[1, 0]
            kf = motor.k_f
            km = motor.k_m

            spin = -motor.dir

            # Total thrust
            MAT[0, i] = kf

            # Roll moment (Mx = y * T)
            MAT[1, i] = y * kf

            # Pitch moment (My = -x * T)
            MAT[2, i] = -x * kf

            # Yaw moment (Mz = spin * km)
            MAT[3, i] = spin * km

        return MAT

    def get_log_definition(self):
        if self.logSchema is None:
            self.logSchema = create_schema(
                fields=[
                    "omega_squared",
                    "flightmode_enable_xy_velocity",
                    "flightmode_enable_z_velocity",
                    "flightmode_enable_position",
                    "flightmode_name",
                ],
                dtypes=[
                    "float64",
                    "bool",
                    "bool",
                    "bool",
                    "str",
                ],
                units=["rad2/s2", "", "", "", ""],
            )
        return self.logSchema

    def export_log(self):
        cfg: "FlightModeConfig" = self.mode.value

        return {
            "omega_squared": self._telem["omega_squared"],
            "flightmode_enable_xy_velocity": cfg.enable_xy_velocity,
            "flightmode_enable_z_velocity": cfg.enable_z_velocity,
            "flightmode_enable_position": cfg.enable_position,
            "flightmode_name": self.mode.name,
        }
