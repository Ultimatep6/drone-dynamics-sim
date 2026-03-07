from dataclasses import dataclass
from typing import List

import numpy as np
from numpy import pi as PI
from numpy import shape
from pydantic import BaseModel, ConfigDict, Field

from quad_sim.math.eulerian import Eulerian
from quad_sim.math.quaternion import Quaternion
from quad_sim.misc.decorators import loggerGroup


@loggerGroup("controller")
class AutoSignal(BaseModel):
    """Empty base model — fields added dynamically."""

    # This allows the model to accept and store fields not defined in the class
    model_config = ConfigDict(extra="allow")

    pass

    @classmethod
    def unpackOmega(cls, omegaDS: np.ndarray):
        if omegaDS.shape == (3, 1):
            omegaDS = omegaDS.T

        elif omegaDS.shape != (1, 3):
            raise TypeError(
                "The omega vector must be a numpy array of shape (3,1) or (1,3)"
            )

        return cls.model_construct(rateRequested=omegaDS.tolist()[0])

    @classmethod
    def unpackQuat(cls, quat: Quaternion):
        euler = quat.to_euler().as_np().flatten().tolist()
        # Create instance normally; 'extra' config handles the rest
        return cls(orientRequested=euler)
