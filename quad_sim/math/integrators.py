import numpy as np

from quad_sim.dynamics.quadcopter import Quadcopter
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.math.rotations import norm_quaternion


# TODO: Develop structure of calculating CM -> Motors -> Wing Tips
def step_forward_euler(body: Quadcopter, inputs: list):
    mass, inertiaVec = body.mass, body.inertia_tensor

    """
    State vector:
        - Inertial Position
        - Linear velocity BF
        - Orientation q
        - angular velocity omega_B
    """
    pos_I, vel_B, q, omega_B = body.get_stateVec()

    """Step 0 - Inputs:
    time
    forces_B
    moments_B
    dt
    """

    t, F_B, M_B, dt = inputs

    """Step 1 - Compute state derivates (outside integrator)
    a_B
    alpha_B
    rate_q
    v_I 
    """
    a_B = compute_aB(mass, F_B, omega_B, vel_B)
    alpha_B = compute_alphaB(inertiaVec, M_B, omega_B)
    rate_q = compute_q_rate(q, omega_B)
    vel_I = BodyFixed.from_EarthFixed(vel_B, q)

    """Step 2 - Integrate angular velocity
    omega_B_i+1 = omega_B_i + alpha_B*dt
    """
    omega_B += alpha_B * dt

    """Step 3 - Update Quaternion
    q_t+1 = q_t*dt
    norm(q_t+1)
    """
    q += rate_q * dt
    q = norm_quaternion(q)

    """Step 4 - Update lin velocity
    v_I = R * v_B
    """
    vel_B += a_B * dt
    vel_I = EarthFixed.from_BodyFixed(vel_B, q)

    """Step 5 - Update position
    r_I+1 = v_I*dt
    """

    r_I = vel_I * dt

    state = [r_I, vel_B, q, omega_B]
    body.update_stateVec(state)

    body.acceleration = EarthFixed.from_BodyFixed(a_B, body.quaternion)
    body.alpha = alpha_B


def compute_aB(
    mass: int | float, F_B: BodyFixed, omega_B: BodyFixed, vel_B: BodyFixed
) -> BodyFixed:
    res = F_B * (1 / mass) - omega_B * vel_B
    res.flag = "acceleration"
    return res


def compute_alphaB(
    inertia: np.ndarray, M_B: BodyFixed, omega_B: BodyFixed
) -> BodyFixed:
    return BodyFixed.from_Array(
        np.linalg.inv(inertia)
        @ (M_B.vec - np.cross(omega_B.vec, (inertia @ omega_B.vec), axis=0)),
        flag="ang_acceleration",
    )


def compute_q_rate(quaternion: np.ndarray, omega_B: BodyFixed) -> np.ndarray:
    p, q, r = omega_B.vec.T.ravel()  # pyright: ignore[reportAssignmentType]
    mat = np.array(
        [
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0],
        ]
    )

    return 0.5 * (mat @ quaternion)
