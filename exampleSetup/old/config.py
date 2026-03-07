from dataclasses import field
from enum import Enum
from typing import Annotated, List, Tuple

import numpy as np
from numpy import pi as PI
from pydantic import BaseModel, Field, field_validator
from pydantic.dataclasses import dataclass

from quad_sim.control.config.autopilotConf import AutopilotConfig
from exampleSetup.default.state import StateVector

# ---------------------------------------
# Settings
# ---------------------------------------


# FUTURE: Add more integrator classes
class IntegratorSettings(Enum):
    FWEuler = "ForwardEuler"
    # BWEuler = 1
    # RK2 = 2


# ---------------------------------------
# Setpoint Limits
# ---------------------------------------


class AttitudeLimits(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)
    # --- Body Frame ---
    max_roll: float | np.floating = np.radians(45)  # rad
    max_pitch: float | np.floating = np.radians(45)  # rad
    max_yaw: float | np.floating = np.radians(180)  # rad


class VelocityLimits(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)
    # --- Inertial Frame ---
    max_vx: float | np.floating = 100.0
    max_vy: float | np.floating = 100.0
    max_vz: float | np.floating = 100.0


class PositionLimits(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)
    # --- Intertial Frame ---
    max_x: float | np.floating = 1000.0
    max_y: float | np.floating = 1000.0
    max_z: float | np.floating = 1000.0


class ThrustConfig(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)
    max_thrust: float | np.floating | None = None
    min_thrust: float | np.floating | None = 0.0


class PhysicalLimits(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)
    attitude_limits: AttitudeLimits = Field(default_factory=AttitudeLimits)
    velocity_limits: VelocityLimits = Field(default_factory=VelocityLimits)
    position_limits: PositionLimits = Field(default_factory=PositionLimits)
    thrust_limits: ThrustConfig = Field(default_factory=ThrustConfig)


# ---------------------------------------
# Drone Layout Configs
# ---------------------------------------


class MotorConfig(BaseModel):
    k_f: Annotated[float, Field(ge=0)] = 3.13e-5
    k_m: Annotated[float, Field(ge=0)] = 7.5e-7

    ang_range: Annotated[
        Tuple[float, float], Field(description="(min_angle, max_angle)")
    ] = (0.0, 600.0**2)

    throttle: int | float = 0.0
    clip_throttle: bool = False

    @field_validator("ang_range")
    def validate_angle_range(cls, v):
        lo, hi = v
        if lo <= 0:
            raise ValueError("ang_range[0] must be > 0")
        if hi <= lo:
            raise ValueError("ang_range[1] must be greater than ang_range[0]")
        return v


class LayoutConfig(BaseModel):
    rotors: List[MotorConfig] = Field(
        default_factory=lambda: [MotorConfig() for _ in range(4)], min_length=4
    )
    arm_length: Annotated[float, Field(gt=0.0)] = 1.0


@dataclass
class RigidConfig:
    mass: int | float = 1.0
    Ixx: float = 1.0
    Iyy: float = 1.0
    Izz: float = 1.0


# ---------------------------------------
# Top Level Vehicle Configs
# ---------------------------------------


@dataclass
class IntegratorConfig:
    setting: IntegratorSettings = IntegratorSettings.FWEuler
    dt: float = 0.1


class VehicleConfig(BaseModel):
    # MetaData
    vehicleID: str = "drone"
    physical_limits: PhysicalLimits = Field(default_factory=PhysicalLimits)
    layout_config: LayoutConfig = Field(default_factory=LayoutConfig)
    rigid_config: RigidConfig = Field(default_factory=RigidConfig)

    # AutoPilot
    pid_config: AutopilotConfig = Field(default_factory=AutopilotConfig)

    # Integrator
    integrator: IntegratorConfig = Field(default_factory=IntegratorConfig)

    # Initial conditions
    init_state: StateVector = StateVector()


# ---------------------------------------
# Top Level Simulation Config
# ---------------------------------------


@dataclass
class SimulatorConfig:
    gravity: bool = True
    environment = None


# ---------------------------------------
# Global Configs
# ---------------------------------------


@dataclass
class NCopterConfig:
    entities: list[VehicleConfig]
    env: SimulatorConfig
