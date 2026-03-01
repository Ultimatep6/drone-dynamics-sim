import numpy as np
from pydantic.dataclasses import dataclass


@dataclass
class AttitudeSetpoint:
    roll_sp: float | None
    pitch_sp: float | None
    yaw_sp: float | None

    def toTelem(self, telemDict: dict):
        arr = self.as_np()
        telemDict["attitude_setpoint"] = arr
        return telemDict

    def as_np(self):
        return np.array(
            [
                self.roll_sp,
                self.pitch_sp,
                self.yaw_sp,
            ],
            dtype=np.float32,
        )


@dataclass
class VelocitySetpoint:
    vx_sp: float | None = 0.0
    vy_sp: float | None = 0.0
    vz_sp: float | None = 0.0

    def toTelem(self, telemDict: dict):
        """Convert the"""
        arr = self.as_np()
        arr[arr is None] = 0.0
        telemDict["velocity_setpoint"] = arr

        return telemDict

    def as_np(self):
        return np.array([[self.vx_sp, self.vy_sp, self.vz_sp]], dtype=np.float32)


@dataclass
class PositionSetpoint:
    x_sp: float | None
    y_sp: float | None
    z_sp: float | None

    def toTelem(self, telemDict: dict):
        telemDict["x_setpoint"] = self.x_sp
        telemDict["y_setpoint"] = self.y_sp
        telemDict["z_setpoint"] = self.z_sp

        return telemDict

    def as_np(self):
        return np.array([[self.x_sp], [self.y_sp], [self.z_sp]], dtype=np.float64)


@dataclass
class ThrustCommand:
    thrust: float | None

    def toTelem(self, telemDict: dict):
        telemDict["thrust_setpoint"] = self.thrust

        return telemDict
