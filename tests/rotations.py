import numpy as np
import pytest

from quad_sim.math.eulerian import Eulerian
from quad_sim.math.quaternion import Quaternion

# ------------------------------------------------------------
# 1. TYPE + SHAPE VALIDATION
# ------------------------------------------------------------


def test_quaternion_inverse_invalid_norm():
    q = Quaternion(2, 0, 0, 0)  # not normalized
    with pytest.raises(ValueError):
        q.inverse()


def test_quaternion_normalize_zero():
    q = Quaternion(0, 0, 0, 0)
    with pytest.raises(ZeroDivisionError):
        q.normalized()


def test_eulerian_from_np_invalid_type():
    with pytest.raises(TypeError):
        Eulerian.from_np([1, 2, 3])


def test_eulerian_from_np_invalid_shape():
    arr = np.zeros((4, 1))
    with pytest.raises(TypeError):
        Eulerian.from_np(arr)


# ------------------------------------------------------------
# 2. BASIC QUATERNION PROPERTIES
# ------------------------------------------------------------


def test_quaternion_norm():
    q = Quaternion(1, 1, 1, 1)
    assert np.isclose(q.norm(), 2.0)


def test_quaternion_normalized():
    q = Quaternion(1, 1, 1, 1)
    qn = q.normalized()
    assert np.isclose(qn.norm(), 1.0)


def test_quaternion_conjugate():
    q = Quaternion(1, 2, 3, 4)
    qc = q.conjugate()
    assert qc == Quaternion(1, -2, -3, -4)


def test_quaternion_inverse():
    q = Quaternion(0, 1, 0, 0).normalized()
    qi = q.inverse()
    assert qi == Quaternion(0, -1, 0, 0)


# ------------------------------------------------------------
# 3. HAMILTON PRODUCT PROPERTIES
# ------------------------------------------------------------


def test_quaternion_identity_multiplication():
    q = Quaternion(0.3, 0.5, -0.2, 0.1).normalized()
    I = Quaternion(1, 0, 0, 0)
    assert (I * q) == q
    assert (q * I) == q


def test_quaternion_non_commutativity():
    q1 = Quaternion(0.7, 0.1, 0.2, 0.1).normalized()
    q2 = Quaternion(0.6, -0.3, 0.1, 0.2).normalized()
    assert (q1 * q2) != (q2 * q1)


# ------------------------------------------------------------
# 4. EULER ↔ QUATERNION CONVERSION
# ------------------------------------------------------------


def test_euler_to_quaternion_and_back():
    e = Eulerian(yaw=0.3, pitch=-0.2, roll=0.1)
    q = e.to_quaternion().normalized()
    e2 = q.to_euler()

    assert np.allclose(e.as_np(), e2.as_np(), atol=1e-6)


def test_eulerian_to_quaternion_simple():
    e = Eulerian(yaw=0.0, pitch=0.0, roll=0.0)
    q = e.to_quaternion().normalized()
    e2 = q.to_euler()

    assert np.allclose(e.as_np(), e2.as_np(), atol=1e-6)


def test_quaternion_from_euler_invalid_type():
    with pytest.raises(TypeError):
        Quaternion.from_euler([1, 2, 3])  # not an Eulerian object


# ------------------------------------------------------------
# 5. SPECIFIC ROTATION CASES
# ------------------------------------------------------------


def test_quaternion_rotation_90deg_roll():
    # 90° roll → quaternion
    e = Eulerian(0, 0, np.pi / 2)
    q = e.to_quaternion().normalized()

    # Rotate x-axis vector (1,0,0)
    v = np.array([1, 0, 0])
    vq = Quaternion(0, *v)

    rotated = q * vq * q.inverse()
    __, x, y, z = rotated.as_np().flatten()

    # After 90° roll, x-axis → x-axis (unchanged)
    assert np.allclose([x, y, z], [1, 0, 0], atol=1e-6)


def test_quaternion_rotation_90deg_pitch():
    e = Eulerian(0, np.pi / 2, 0)
    q = e.to_quaternion().normalized()

    v = np.array([1, 0, 0])
    vq = Quaternion(0, *v)

    rotated = q * vq * q.inverse()
    _, x, y, z = rotated.as_np().flatten()

    # After 90° pitch, x-axis → z-axis
    assert np.allclose([x, y, z], [0, 0, -1], atol=1e-6)
