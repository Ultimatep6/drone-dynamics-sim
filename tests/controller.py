import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import solve

from quad_sim.config import VehicleConfig
from quad_sim.control.RC import RC
from quad_sim.control.systems.AutoPilot import AutoPilot
from quad_sim.simulation.drone import Drone
from quad_sim.utils.loggerV2 import NCopterLogger

d = Drone(VehicleConfig(vehicleID="drone_001"))
c = NCopterLogger("test.hdf5")
c.register_drone(d)
c.step()

r = RC(0.0, -1.0, 0.0, 1.0)
d.controller.__RC = r
d.controller.stepPID(d.controller.rc_to_command())

c.step()
c.finalize()
