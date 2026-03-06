from typing import Optional
from pydantic import BaseModel, ConfigDict

class Setpoints(BaseModel):
    """
    Universal data container for controller 
    setpoints which are created by the PilotBase and interpretted by the AllocatorBase

    For user-defined extensions, any custom setpoint can be easily created with Setpoint(custom=1.0) (ex.)
    """
    # --- Standard, known setpoints --- #
    roll_angle: Optional[float] = None
    pitch_angle: Optional[float] = None
    yaw_angle: Optional[float] = None
    roll_rate: Optional[float] = None
    pitch_rate: Optional[float] = None
    yaw_rate: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    vz: Optional[float] = None
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    # Body Frame z thrust (upwards)
    thrustz: Optional[float] = None

    # Allows for custom 
    model_config = ConfigDict(extra='allow')
