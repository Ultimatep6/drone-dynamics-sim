"""
This is a script for testing how the reference frames are converted
Suppose we have a quad copter with copter positons
[1,0,0] [-1,0,0] [0,1,0] [0,-1,0] in BF frame
"""

import numpy as np

from quad_sim.math.references.bodyFixed import BodyFixed

pos1 = np.array([[1], [0], [0]])
quad1 = BodyFixed(pos=pos1)

quad1.orientation = np.array([[np.pi / 2, 0, 0]])

print(quad1)

print(quad1._toearthFixed())
