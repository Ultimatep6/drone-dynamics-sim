from pydantic import BaseModel, Field, ConfigDict
from quad_sim.references.earthFixed import EarthFixed
from quad_sim.references.bodyFixed import BodyFixed
from quad_sim.orientation.quaternion import Quaternion

import numpy as np

class StateVector(BaseModel):
    # Allows for custom 
    model_config = ConfigDict(extra='allow', arbitrary_types_allowed=True)

    position: EarthFixed = Field(
        default_factory=lambda: EarthFixed.from_Array(np.zeros((3, 1)))
    )
    velocity: BodyFixed = Field(
        default_factory=lambda: BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity")
    )
    quaternion: Quaternion = Field(
        default_factory=lambda: Quaternion(1.0, 0.0, 0.0, 0.0)
    )
    omega: BodyFixed = Field(
        default_factory=lambda: BodyFixed.from_Array(
            np.zeros((3, 1)), flag="ang_velocity"
        )
    )
    acceleration: BodyFixed = Field(
        default_factory=lambda: BodyFixed.from_Array(
            np.zeros((3, 1)), flag="acceleration"
        )
    )
    alpha: BodyFixed = Field(
        default_factory=lambda: BodyFixed.from_Array(
            np.zeros((3, 1)), flag="ang_acceleration"
        )
    )

    