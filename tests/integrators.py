"""
This document is all about running tests on the integrator file
"""

import numpy as np
import pytest

from quad_sim.math.integrators import *
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed


# TODO: Setup full integrator tests
def test_quaternion_rate():
    q = np.array([[np.cos(np.pi / 4)], [0], [np.sin(np.pi / 4)], [0]])
    omega = BodyFixed(0, 0, 10, flag="ang_velocity")
    assert np.allclose(
        compute_q_rate(q, omega),
        np.array([[0], [5 * np.sqrt(2) / 2], [0], [5 * np.sqrt(2) / 2]]),
    )
