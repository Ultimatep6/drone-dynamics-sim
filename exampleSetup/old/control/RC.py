from pydantic import BaseModel, ConfigDict, Field

class RC(BaseModel):
    model_config = ConfigDict(validate_assignment=True)

    Rx: float = Field(default=0.0, ge=-1.0, le=1.0) # Roll or Yaxis velocity (horizontal)
    Ry: float = Field(default=0.0, ge=-1.0, le=1.0) # Pitch or Xaxis velocity (fowards)
    Lx: float = Field(default=0.0, ge=-1.0, le=1.0) # Yaw
    Ly: float = Field(default=0.0, ge=-1.0, le=1.0) # Thrust or Zaxis velocity (vertical)

    def updateInputs(self, axis, value):
        """
        Update one RC axis, or all if axis is False.
        """
        if hasattr(self, axis):
            setattr(self, axis, value)
        elif not axis:
            self.Rx = self.Ry = self.Lx = self.Ly = value
        else:
            raise ValueError(f"axis must be one of {self.__dict__.keys()} or False")
