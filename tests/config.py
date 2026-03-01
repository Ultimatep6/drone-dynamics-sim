from dataclasses import astuple

import numpy as np

from quad_sim.config import *
from quad_sim.simulation.simulator import nCopterSimulator

bettyConfig = VehicleConfig(
    vehicleID="betty",
    attitude_limits=AttitudeLimits(
        max_roll=30 * np.pi / 180,
        max_pitch=30 * np.pi / 180,
        max_yaw=120 * np.pi / 180,
    ),
    layout_config=LayoutConfig(),
    rigid_config=RigidConfig(),
)

envConfig = SimulatorConfig(dt=0.1)

config = NCopterConfig(entities=[bettyConfig], env=envConfig)

sim = nCopterSimulator(config)
print(sim.entities)
