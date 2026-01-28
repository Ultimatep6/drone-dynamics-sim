"""
This is a test to check that quaternion rotations are made correctly
"""

from csv import Error
from sys import exception
from typing import Type

import numpy as np
import pytest
from numpy import cos as c
from numpy import pi as PI
from numpy import sin as s
from numpy import tan as t

from quad_sim.math.rotations import (
    inverse_quaternion,
    norm_quaternion,
    quaternion_rotation,
)

"""
Test 1 : Check that quaternion_rotation is fail proof
"""


def test_quaternion_rotation_invalid_type():
    q = [1, 0, 0, 0]
    e = np.array([[1], [0], [0], [0]])

    with pytest.raises(TypeError):
        quaternion_rotation(e, q)


def test_quaternion_inverse_invalid_type():
    q = [1, 0, 0, 0]

    with pytest.raises(TypeError):
        inverse_quaternion(q)


def test_quaternion_norm_invalid_type():
    q = [1, 0, 0, 0]

    with pytest.raises(TypeError):
        norm_quaternion(q)


def test_quaternion_rotation_invalid_shape():
    q = np.zeros((3, 1))
    e = np.array([[1], [0], [0], [0]])

    with pytest.raises(TypeError):
        quaternion_rotation(e, q)


def test_quaternion_inverse_invalid_shape():
    q = np.zeros((3, 1))

    with pytest.raises(TypeError):
        inverse_quaternion(q)


def test_quaternion_norm_invalid_shape():
    q = np.zeros((3, 1))

    with pytest.raises(TypeError):
        norm_quaternion(q)


def test_quaternion_rotation_invalid_quaternion():
    q = np.ones((4, 1))  # not normalized
    e = np.array([[1], [0], [0], [0]])

    with pytest.raises(ValueError):
        quaternion_rotation(q, e)


"""
Test 2: Test that the calculations are right
a) Only the identity matrix is commutative
b) Simple calculation
c) Test that commutativity is not valid in most cases
d) Test that the inverse works
e) Test that norm works
"""


def test_quaternion_assersion():
    q = np.array([[1], [0], [0], [0]])
    e = np.array([[0.8], [0.1], [0.2], [0.3]])

    assert (
        quaternion_rotation(q, e, debug=False) == quaternion_rotation(e, q, debug=False)
    ).all()


def test_quaternion_rotation_calculation():
    q = np.array([[c(PI / 4)], [s(PI / 4)], [0], [0]])
    e = np.array([[c(PI / 4)], [0], [s(PI / 4)], [0]])

    assert np.allclose(
        quaternion_rotation(q, e, debug=True), np.full((4, 1), 0.5), atol=1e-6
    )


def test_quaternion_commutativity():
    q = np.array([[c(PI / 4)], [s(PI / 4)], [0], [0]])
    e = np.array([[c(PI / 4)], [0], [s(PI / 4)], [0]])

    assert (
        quaternion_rotation(q, e, debug=False) != quaternion_rotation(e, q, debug=False)
    ).any()


def test_quaternion_inverse_calc():
    q = np.zeros((4, 1), dtype=np.float32)
    q[1, 0] = 1.0

    expected = np.array([[0], [-1], [0], [0]], dtype=np.float32)

    assert np.allclose(inverse_quaternion(q), expected)


def test_quaternion_norm_calc():
    q = np.ones((4, 1))

    assert (norm_quaternion(q) == np.full((4, 1), 0.5)).all()
