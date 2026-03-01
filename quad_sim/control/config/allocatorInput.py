import numpy as np
from pydantic.dataclasses import dataclass

from quad_sim.math.quaternion import Quaternion


@dataclass
class AllocatorInput:
    thrust: int | float
    momentX: int | float
    momentY: int | float
    momentZ: int | float

    def as_np(self):
        return np.array([[self.thrust, self.momentX, self.momentY, self.momentZ]]).T
