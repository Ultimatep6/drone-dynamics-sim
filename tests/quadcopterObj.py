import numpy as np
from numpy import pi as PI

from quad_sim.dynamics.quadcopter import Quadcopter
from quad_sim.math.integrators import *
from quad_sim.math.rotations import _eulerian_to_quaternion

orientation = np.array([[0], [PI / 4], [0]])
q = _eulerian_to_quaternion(orientation)
print(q)

drone1 = Quadcopter(mass=0.1, Ixx=1, Iyy=1, Izz=1, arm_length=2)
# q = np.array([[np.cos(PI / 4)], [0], [np.sin(PI / 4)], [0]])
drone1.quaternion = q
drone1.setThrottle(10, "all")
print(drone1)

thrust, min, max = drone1.max_forces()
print(f"Thrust: {thrust.vec}")
print(f"Min: {min}")
print(f"Max: {max}")


drone1.print_layout()
