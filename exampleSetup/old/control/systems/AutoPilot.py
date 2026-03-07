"""
An autopilot controller.
Arguments:
    - Thrust cmd
    - Roll cmd
    - Pitch cmd
    - Yaw cmd
    - Velocity X
    - Velocity Y
    - Velocity Z

Ouputs:
    - Motor N cmd

Settings:
--- Stabalized Mode ---
The RC roll cmd shall control roll angle
The RC yaw cmd shall control yaw angle
The RC pitch cmd shall control pitch angle
The Rc throttle cmd shall go directly to command allocation (omega solver)


--- Altitude Mode ---
The RC roll cmd shall control roll angle
The RC yaw cmd shall control yaw angle
The RC pitch cmd shall control pitch angle
The Rc throttle cmd shall control the vertical velocity


--- Position Mode ---
The RC roll cmd shall control Y-axis velocity
The RC yaw cmd shall control yaw angle
The RC pitch cmd shall control X-axis velocity
The Rc throttle cmd shall control the vertical velocity

--- Trajectory Mode ---
....
"""

from __future__ import annotations

import numpy as np

from quad_sim.control.config.autopilotConf import AutopilotConfig
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.control.config.setpoints import (
    AttitudeSetpoint,
    PositionSetpoint,
    ThrustCommand,
    VelocitySetpoint,
)
from quad_sim.math.eulerian import Eulerian
from quad_sim.math.quaternion import Quaternion
from quad_sim.misc.decorators import subsystem
from quad_sim.utils.logFuncs import create_schema
from quad_sim.utils.loggerV2 import NCopterLogger

# TODO: Add to environment
GRAVITY = 9.81


# TODO: Polish the autopilot and setup tests
@subsystem("Controller")
class AutoPilot:
    def __init__(
        self,
        drone: "Drone",
        logSchema: dict | None = None,
        pidConfig: AutopilotConfig | None = None,
    ) -> None:
        self.drone = drone

        self.logSchema = logSchema
        if self.logSchema is None:
            self.logSchema = self.get_log_definition()

        self._telem = {field: 0 for field in self.logSchema.keys()}
        print(self._telem)

        if pidConfig is None:
            self.config = AutopilotConfig()
        else:
            self.config = pidConfig

        self.mode = self.config.mode

        self.att_pidConfig = (
            self.config.gains.roll,
            self.config.gains.pitch,
            self.config.gains.yaw,
        )
        self.rate_pidConfig = (
            self.config.gains.roll_rate,
            self.config.gains.pitch_rate,
            self.config.gains.yaw_rate,
        )
        self.vel_pidConfig = (
            self.config.gains.vx,
            self.config.gains.vy,
            self.config.gains.vz,
        )
        self.pos_pidConfig = (
            self.config.gains.px,
            self.config.gains.py,
            self.config.gains.pz,
        )

    def stepPID(self, setpoints: dict, logger: NCopterLogger | None = None):
        state = self.drone.getStateVec()

        if self.mode == FlightMode.STABILIZED:
            tau_x, tau_y, tau_z = self.__computeTorques(setpoints)

            thrust = setpoints["thrust"].thrust

            self._telem["tau_x_setpoint"] = tau_x
            self._telem["tau_y_setpoint"] = tau_y
            self._telem["tau_z_setpoint"] = tau_z
            # Earth Frame
            self._telem["thrust_setpoint"] = thrust

            return [thrust, tau_x, tau_y, tau_z]

        elif self.mode == FlightMode.ALTITUDE:
            # Get torques
            tau_x, tau_y, tau_z = self.__computeTorques(setpoints)

            # Get the current velocity
            vz_ms = self.drone.state.velocity.vec[2, 0]
            v_sp = setpoints["velocity"].as_np()

            pid_velz = self.config.gains.controllers["vz"]

            # Convert the thrust to vertical velocity
            thrust_adj = -pid_velz.compute(v_sp.flatten()[2], vz_ms)
            hover_thrust = GRAVITY * self.drone.model.mass
            thrust = hover_thrust + thrust_adj

            thrust = max(0.0, thrust)

            self._telem["tau_x_setpoint"] = tau_x
            self._telem["tau_y_setpoint"] = tau_y
            self._telem["tau_z_setpoint"] = tau_z
            self._telem["thrust_setpoint"] = thrust

            return [thrust, tau_x, tau_y, tau_z]
            return NotImplementedError

    def rc_to_command(self, rc: "RC"):  # -> dict[str, float | EarthFixed | BodyFixed]:
        fm = self.mode.value  # FlightModeConfig
        cfg = self.drone.limits

        # --- Attitude commands (always available) ---
        roll_sp = rc.Rx * cfg.attitude_limits.max_roll
        pitch_sp = rc.Ry * cfg.attitude_limits.max_pitch
        yaw_sp = rc.Lx * cfg.attitude_limits.max_yaw

        # --- Velocity commands ---
        if fm.enable_xy_velocity:
            vx_sp = rc.RC.Rx * cfg.velocity_limits.max_vx
            vy_sp = rc.RC.Ry * cfg.velocity_limits.max_vy
        else:
            vx_sp = None
            vy_sp = None

        if fm.enable_z_velocity:
            vz_sp = -rc.Ly * cfg.velocity_limits.max_vz
        else:
            vz_sp = None

        # --- Position commands ---
        # if fm.enable_position:
        # Typically you integrate velocity commands or map sticks to position increments
        # x_sp = self.rc.Rx * cfg.position_limits.max_x
        # y_sp = self.rc.Ry * cfg.position_limits.max_y
        # z_sp = self.rc.Ly * cfg.position_limits.max_z
        # print("huh")
        # else:
        x_sp = y_sp = z_sp = None

        # --- Thrust command (only in stabilized ) ---
        if not fm.enable_z_velocity:
            thrust = rc.Ly * cfg.thrust_limits.max_thrust
        else:
            thrust = None

        # att = _eulerian_to_quaternion(np.array([[roll_sp], [pitch_sp], [yaw_sp]]))
        # vel = EarthFixed()

        att = AttitudeSetpoint(roll_sp, pitch_sp, yaw_sp)
        vel = VelocitySetpoint(vx_sp, vy_sp, vz_sp)
        pos = PositionSetpoint(x_sp, y_sp, z_sp)

        # Inertial Frame
        thr = ThrustCommand(thrust)

        telem = self._telem
        # Record the telemetry
        # attitude
        telem = att.toTelem(telem)
        # Velocity_IF -> Velocity_BF
        telem = vel.toTelem(telem)
        # position
        telem = pos.toTelem(telem)
        # thrust
        telem = thr.toTelem(telem)

        self._telem = telem
        return {
            "attitude": att,
            "velocity": vel,
            "position": pos,
            "thrust": thr,
        }

    def __computeTorques(self, setpoints: dict):
        # Convert quaternion position to rate setpoints
        setpoint = setpoints["attitude"]

        # Convert the eulerian setpoint to a quaternion
        e = Eulerian(
            yaw=setpoint.yaw_sp, pitch=setpoint.pitch_sp, roll=setpoint.roll_sp
        )
        quat_sp = e.to_quaternion()

        # Get current orientation of body and inverse it
        meas = self.drone.state.quaternion
        inv_meas = meas.inverse()

        # Compute the error
        q_err = quat_sp * inv_meas

        # Extract the quaternion error vector and convert
        att_err_vec = 2.0 * q_err.as_np()[1:, :]

        # Get the gain vector for att -> rate
        K_ATT = np.array([config.kp for config in self.att_pidConfig])[None, :]

        # Compute the desired rate
        omega_sp = K_ATT * att_err_vec

        # Save the desired omegas in telem
        self._telem["omega_setpoint"] = omega_sp

        # Get the current rate of the body
        omega_ms = self.drone.state.omega.vec

        pid_p = self.config.gains.controllers["roll_rate"]
        pid_q = self.config.gains.controllers["pitch_rate"]
        pid_r = self.config.gains.controllers["yaw_rate"]

        tau_x = pid_p.compute(
            omega_sp[0, 0],
            omega_ms[0, 0],
        )
        tau_y = pid_q.compute(
            omega_sp[1, 0],
            omega_ms[1, 0],
        )
        tau_z = pid_r.compute(
            omega_sp[2, 0],
            omega_ms[2, 0],
        )

        return tau_x, tau_y, tau_z

    def get_log_definition(self):
        if self.logSchema is None:
            # Default Setting
            self.logSchema = create_schema(
                fields=[
                    "roll_setpoint",
                    "pitch_setpoint",
                    "yaw_setpoint",
                    "omega_setpoint",
                    "velocity_setpoint",
                    "thrust_setpoint",
                ],
                dtypes=[
                    "float32",
                    "float32",
                    "float32",
                    "float64",
                    "float32",
                    "float32",
                ],
                units=["rad", "rad", "rad", "rad", "m/s", "N"],
            )

        return self.logSchema

    def export_log(self):
        return {
            "roll_setpoint": self._telem["roll_setpoint"],
            "pitch_setpoint": self._telem["pitch_setpoint"],
            "yaw_setpoint": self._telem["yaw_setpoint"],
            "omega_setpoint": self._telem["omega_setpoint"],
            "velocity_setpoint": self._telem["velocity_setpoint"],
            "thrust_setpoint": self._telem["thrust_setpoint"],
        }
