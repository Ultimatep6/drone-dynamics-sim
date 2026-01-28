import numpy as np
from numpy import pi as PI

from quad_sim.dynamics.quadcopter import Quadcopter
from quad_sim.math.references.earthFixed import EarthFixed

pos = EarthFixed(X=0, Y=0, Z=0)
drone1 = Quadcopter(mass=5, Ixx=1, Iyy=1, Izz=1, rotors=4, arm_length=2)
drone1.quaternion = np.array([[np.cos(PI / 4)], [0], [np.sin(PI / 4)], [0]])
print(drone1)
drone1.position += EarthFixed(X=2, Y=3, Z=0)
print(drone1)
