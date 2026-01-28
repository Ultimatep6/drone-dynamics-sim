from tkinter import E

import numpy as np
import pytest
from numpy import pi as PI

from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed

"""
TESTING REFERENCE CLASSES

Test 1: Fail proofing initialization
"""


def test_bodyFixed_init():
    with pytest.raises(TypeError):
        tup = ("a", "b", "c")
        BodyFixed(*tup)

    with pytest.raises(TypeError):
        tup = ("a", 2, "b")
        BodyFixed(*tup)

    with pytest.raises(TypeError):
        arr = 5
        BodyFixed.from_Array(arr)

    with pytest.raises(TypeError):
        "Check quaternion type"
        arr = np.ones((3, 1))
        b = EarthFixed.from_Array(arr)
        pos = EarthFixed(0, 0, 10)
        quat = "a"
        BodyFixed.from_EarthFixed(b, quat, pos)

    with pytest.raises(ValueError):
        "Check quaternion value"
        arr = np.ones((3, 1))
        b = EarthFixed.from_Array(arr)
        pos = EarthFixed(0, 0, 10)
        quat = np.ones((4, 1))
        BodyFixed.from_EarthFixed(b, quat, pos)

    with pytest.raises(TypeError):
        "Test EarthFixed type"
        quat = np.array([[1], [0], [0], [0]])
        pos = EarthFixed(0, 0, 10)
        a = "a"
        BodyFixed.from_EarthFixed(a, quat, pos)

    with pytest.raises(TypeError):
        "Test CM type"
        quat = np.array([[1], [0], [0], [0]])
        pos = BodyFixed(0, 0, 0)
        a = EarthFixed(0, 1, 10)
        BodyFixed.from_EarthFixed(a, quat, pos)


def test_earthFixed_init():
    with pytest.raises(TypeError):
        tup = ("a", "b", "c")
        EarthFixed(*tup)

    with pytest.raises(TypeError):
        tup = ("a", 2, "b")
        EarthFixed(*tup)

    with pytest.raises(TypeError):
        arr = 5
        EarthFixed.from_Array(arr)

    with pytest.raises(TypeError):
        "Check quaternion type"
        arr = np.ones((3, 1))
        pos = EarthFixed(0, 0, 10)
        b = BodyFixed.from_Array(arr)
        quat = "a"
        EarthFixed.from_BodyFixed(b, quat, pos)

    with pytest.raises(ValueError):
        "Check quaternion value"
        arr = np.ones((3, 1))
        pos = EarthFixed(0, 0, 10)
        b = BodyFixed.from_Array(arr)
        quat = np.ones((4, 1))
        EarthFixed.from_BodyFixed(b, quat, pos)

    with pytest.raises(TypeError):
        "Test BodyFixed type"
        quat = np.array([[1], [0], [0], [0]])
        pos = EarthFixed(0, 0, 10)
        a = "a"
        EarthFixed.from_BodyFixed(a, quat, pos)

    with pytest.raises(TypeError):
        "Test CM type"
        quat = np.array([[1], [0], [0], [0]])
        pos = "a"
        a = BodyFixed(0, 1, 10)
        EarthFixed.from_BodyFixed(a, quat, pos)


"""
Test 2: Checking Math calculations
"""


def test_bodyFixed_math():
    """
    Test that position for 90 deg yaw
    """
    p = BodyFixed(1, 0, 0)
    c = EarthFixed(10, 5, 2)
    r = np.array([[np.cos(PI / 4)], [0], [0], [np.sin(PI / 4)]])

    assert np.allclose(
        EarthFixed.from_BodyFixed(p, r, c).vec, np.array([[10], [6], [2]])
    )

    """
    Test that a thrust vector for 90 deg yaw
    """
    t = BodyFixed(0, 0, -5, flag="force")
    r = r

    assert np.allclose(EarthFixed.from_BodyFixed(t, r).vec, np.array([[0], [0], [-5]]))


def test_earthFixed_math():
    """
    Test a global motor position
    """
    p = EarthFixed(10, 6, 2)
    c = EarthFixed(10, 5, 2)
    r = np.array([[np.cos(PI / 4)], [0], [0], [np.sin(PI / 4)]])

    assert np.allclose(
        BodyFixed.from_EarthFixed(p, r, c).vec, np.array([[1], [0], [0]])
    )

    """
    Test a velocity vector
    """
    v = EarthFixed(0, 0, -5, flag="velocity")
    r = r

    assert np.allclose(BodyFixed.from_EarthFixed(v, r).vec, np.array([[0], [0], [-5]]))


""" 
Test 3: 
Testing the add and mult operators
"""


def test_add_operator():
    with pytest.raises(TypeError):
        a = BodyFixed(0, 0, 0)
        b = "a"

        a + b
    with pytest.raises(TypeError):
        a = BodyFixed(0, 0, 0)
        b = BodyFixed(0, 0, 0, flag="velocity")

        a + b

    with pytest.raises(TypeError):
        a = EarthFixed(0, 0, 0)
        b = "a"

        a + b
    with pytest.raises(TypeError):
        a = EarthFixed(0, 0, 0)
        b = EarthFixed(0, 0, 0, flag="velocity")

        a + b

    a = BodyFixed(0, 0, 0)
    b = BodyFixed(0, 0, 1)
    assert a + b == BodyFixed(0, 0, 1)

    a = BodyFixed(0, 0, 0, flag="velocity")
    b = BodyFixed(0, 0, 1, flag="velocity")
    assert a + b == BodyFixed(0, 0, 1, flag="velocity")

    a = EarthFixed(0, 0, 0)
    b = EarthFixed(0, 0, 1)
    assert a + b == EarthFixed(0, 0, 1)

    a = EarthFixed(0, 0, 0, flag="velocity")
    b = EarthFixed(0, 0, 1, flag="velocity")
    assert a + b == EarthFixed(0, 0, 1, flag="velocity")


def test_mult_operator():
    with pytest.raises(TypeError):
        a = BodyFixed(0, 0, 0)
        b = "a"

        a * b

    with pytest.raises(TypeError):
        a = EarthFixed(0, 0, 0)
        b = "a"

        a * b

    a = BodyFixed(0, 1, 0)
    b = BodyFixed(0, 0, 1)
    assert a * b == BodyFixed(1, 0, 0)

    a = BodyFixed(0, 1, 0, flag="velocity")
    b = BodyFixed(0, 0, 1, flag="velocity")
    assert a * b == BodyFixed(1, 0, 0, flag="velocity")

    a = BodyFixed(1, 1, 1)
    b = 2
    assert a * b == BodyFixed(2, 2, 2)

    a = EarthFixed(0, 1, 0)
    b = EarthFixed(0, 0, 1)
    assert a * b == EarthFixed(1, 0, 0)

    a = EarthFixed(0, 1, 0, flag="velocity")
    b = EarthFixed(0, 0, 1, flag="velocity")
    assert a * b == EarthFixed(1, 0, 0, flag="velocity")

    a = EarthFixed(1, 1, 1)
    b = 2
    assert a * b == EarthFixed(2, 2, 2)
