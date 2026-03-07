from __future__ import annotations

from typing import List

import numpy as np
from pydantic import BaseModel, Field

from exampleSetup.default.config import MotorConfig
from quad_sim.dynamics.motors import PropMotor
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.misc.decorators import loggerGroup


@loggerGroup("allocator")
class MotorSignal(BaseModel):
    motorRPM: List[float] = Field(min_length=4)

    @classmethod
    def unpackMotors(cls, motors: List["PropMotor"]):
        return cls(motorRPM=[m.throttle for m in motors])

    def __str__(self) -> str:
        ostr = ""
        for i, motor in enumerate(self.motorRPM):
            ostr += f"Motor_{i} : {motor}\n"
        return ostr
