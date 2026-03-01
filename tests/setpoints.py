import pytest
from pydantic import ValidationError

from quad_sim.control.config.setpoints import (
    AttitudeSetpoint,
    PositionSetpoint,
    VelocitySetpoint,
)


# BUG: Currently the dataclasses do not enforce the type of attributes
def testInitATT():
    roll, pitch, yaw = 1.0, 10.0, "a"
    with pytest.raises(ValidationError):
        AttitudeSetpoint(roll, pitch, yaw)


def testInitPOS():
    roll, pitch, yaw = 1.0, 10.0, "a"
    with pytest.raises(ValidationError):
        PositionSetpoint(roll, pitch, yaw)
