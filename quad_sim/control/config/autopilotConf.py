from pydantic import BaseModel, Field
from pydantic.dataclasses import dataclass

from quad_sim.control.config.flightMode import FlightMode
from quad_sim.control.config.gains import GainsConf, PidGains

# ------------------------------------------------------------
# Top-Level Autopilot Configuration
# ------------------------------------------------------------


class AutopilotConfig(BaseModel):
    gains: GainsConf = Field(default_factory=GainsConf)
    mode: FlightMode = FlightMode.STABILIZED
