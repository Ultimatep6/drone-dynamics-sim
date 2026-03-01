from typing import Annotated

import numpy as np
from pydantic import AfterValidator, BaseModel, Field

from quad_sim.dynamics.state import StateVector
from quad_sim.misc.decorators import loggerGroup


def validate_shape(shape: tuple):
    def validator(v: np.ndarray) -> np.ndarray:
        if v.shape != shape:
            raise ValueError(f"Expected shape {shape}, got {v.shape}")
        if not np.isfinite(v).all():
            raise ValueError("Array contains NaN or Inf")
        return v

    return AfterValidator(validator)


def validate_unit_norm(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if not np.isclose(norm, 1.0, atol=1e-5):
        raise ValueError(f"Quaternion must be unit norm (got {norm})")
    return v


# Define reusable types
Vec3 = Annotated[np.ndarray, validate_shape((3, 1))]
Quat = Annotated[np.ndarray, validate_shape((4, 1)), AfterValidator(validate_unit_norm)]


@loggerGroup("state")
class StateSignal(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)

    position: Vec3
    velocity: Vec3
    quaternion: Quat
    omega: Vec3

    @classmethod
    def unpackState(cls, state: "StateVector"):
        return cls(
            position=state.position.vec,
            velocity=state.velocity.vec,
            quaternion=state.quaternion.as_np(),
            omega=state.omega.vec,
        )

    def __str__(self) -> str:
        return f"{self.model_dump()}"



