from dataclasses import dataclass
from enum import Enum

# ------------------------------------------------------------
# Flight Mode Behavior
# ------------------------------------------------------------


@dataclass
class FlightModeConfig:
    enable_xy_velocity: bool
    enable_z_velocity: bool
    enable_position: bool


class FlightMode(Enum):
    STABILIZED = FlightModeConfig(
        enable_xy_velocity=False, enable_z_velocity=False, enable_position=False
    )
    ALTITUDE = FlightModeConfig(
        enable_xy_velocity=False, enable_z_velocity=True, enable_position=False
    )
    POSITION = FlightModeConfig(
        enable_xy_velocity=True, enable_z_velocity=True, enable_position=True
    )
