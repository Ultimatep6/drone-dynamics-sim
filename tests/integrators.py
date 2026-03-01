"""
This document is all about running tests on the integrator file
"""

import numpy as np
import pytest

from quad_sim.dynamics.quadcopter import Quadcopter
from quad_sim.math.integrators import compute_aB, compute_alphaB
from quad_sim.math.references.bodyFixed import BodyFixed

# ---------------------------------------------------------------------------
# Tests for compute_aB
# ---------------------------------------------------------------------------


def test_compute_aB_correct_physics():
    mass = 2.0
    F_B = BodyFixed.from_Array(np.array([[4.0], [0.0], [0.0]]), flag="force")
    omega_B = BodyFixed.from_Array(np.array([[0.0], [0.0], [1.0]]), flag="ang_velocity")
    vel_B = BodyFixed.from_Array(np.array([[0.0], [2.0], [0.0]]), flag="velocity")

    expected = np.array([[4.0], [0.0], [0.0]])

    a = compute_aB(mass, F_B, omega_B, vel_B)
    assert np.allclose(a.vec, expected)
    assert a.flag == "acceleration"


def test_compute_aB_invalid_mass_type():
    with pytest.raises(TypeError):
        compute_aB(
            "bad",
            BodyFixed.from_Array(np.zeros((3, 1)), flag="force"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity"),
        )


def test_compute_aB_negative_mass():
    with pytest.raises(ValueError):
        compute_aB(
            -1.0,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="force"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity"),
        )


def test_compute_aB_invalid_F_type():
    with pytest.raises(TypeError):
        compute_aB(
            1.0,
            "not a BodyFixed",
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity"),
        )


def test_compute_aB_invalid_omega_type():
    with pytest.raises(TypeError):
        compute_aB(
            1.0,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="force"),
            "bad",
            BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity"),
        )


def test_compute_aB_invalid_vel_type():
    with pytest.raises(TypeError):
        compute_aB(
            1.0,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="force"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
            "bad",
        )


def test_compute_aB_nonfinite_values():
    bad = BodyFixed.from_Array(np.array([[np.nan], [0.0], [0.0]]), flag="force")
    with pytest.raises(ValueError):
        compute_aB(
            1.0,
            bad,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="velocity"),
        )


# ---------------------------------------------------------------------------
# Tests for compute_alphaB
# ---------------------------------------------------------------------------


def test_compute_alphaB_correct_physics():
    inertia = np.diag([2.0, 3.0, 4.0])
    M_B = BodyFixed.from_Array(np.array([[1.0], [2.0], [3.0]]), flag="moment")
    omega_B = BodyFixed.from_Array(np.array([[0.1], [0.2], [0.3]]), flag="ang_velocity")

    rhs = M_B.vec - np.cross(omega_B.vec, inertia @ omega_B.vec, axis=0)
    expected = np.linalg.solve(inertia, rhs)

    alpha = compute_alphaB(inertia, M_B, omega_B)
    assert np.allclose(alpha.vec, expected)
    assert alpha.flag == "ang_acceleration"


def test_compute_alphaB_invalid_inertia_type():
    with pytest.raises(TypeError):
        compute_alphaB(
            "bad",
            BodyFixed.from_Array(np.zeros((3, 1)), flag="moment"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
        )


def test_compute_alphaB_invalid_inertia_shape():
    with pytest.raises(ValueError):
        compute_alphaB(
            np.zeros((2, 2)),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="moment"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
        )


def test_compute_alphaB_nonfinite_inertia():
    inertia = np.array([[1.0, 0.0, 0.0], [0.0, np.nan, 0.0], [0.0, 0.0, 3.0]])
    with pytest.raises(ValueError):
        compute_alphaB(
            inertia,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="moment"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
        )


def test_compute_alphaB_singular_inertia():
    inertia = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError):
        compute_alphaB(
            inertia,
            BodyFixed.from_Array(np.zeros((3, 1)), flag="moment"),
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
        )


def test_compute_alphaB_invalid_M_type():
    with pytest.raises(TypeError):
        compute_alphaB(
            np.eye(3),
            "bad",
            BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity"),
        )


def test_compute_alphaB_invalid_omega_type():
    with pytest.raises(TypeError):
        compute_alphaB(
            np.eye(3), BodyFixed.from_Array(np.zeros((3, 1)), flag="moment"), "bad"
        )


def test_compute_alphaB_nonfinite_vector():
    bad = BodyFixed.from_Array(np.array([[np.nan], [0.0], [0.0]]), flag="moment")
    with pytest.raises(ValueError):
        compute_alphaB(
            np.eye(3), bad, BodyFixed.from_Array(np.zeros((3, 1)), flag="ang_velocity")
        )
