import numpy as np
from pydantic import BaseModel, Field
from pydantic.dataclasses import dataclass

from quad_sim.math.quaternion import Quaternion
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.math.rotations import _get_body_to_inertial
from quad_sim.misc.decorators import subsystem
from quad_sim.utils.logFuncs import create_schema


@subsystem("State")
class StateVector(BaseModel):
    model_config = dict(arbitrary_types_allowed=True)

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

    logSchema: dict | None = Field(default_factory=lambda: None)

    def get_log_definition(self):
        """
        Returns the standard logSchema
        """
        if self.logSchema is None:
            self.logSchema = create_schema(
                fields=["position", "velocity", "eulerianOrientation", "omega"],
                dtypes=["float32", "float64", "float64", "float32"],
                units=["m", "m/s", "rad", "rad/s"],
            )
            return self.logSchema
        return self.logSchema

    def export_log(self):
        return {
            "position": self.position.vec.T,
            "velocity": (
                _get_body_to_inertial(self.quaternion.as_np()) @ self.velocity.vec
            ).T,
            "eulerianOrientation": self.quaternion.to_euler().as_np().T,
            "omega": self.omega.vec.T,
        }

    def __str__(self):
        return f"Position: {self.position.vec.T}\nVelocity: {self.velocity.vec.T}\nOrientation Quat: {self.quaternion.as_np().T}\nOmega: {self.omega.vec.T}"
