from __future__ import annotations

from typing import Tuple

import numpy as np

from exampleSetup.default.config import IntegratorConfig
from quad_sim.math.quaternion import Quaternion
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.utils.logFuncs import create_schema


# FUTURE: Develop structure of calculating CM -> Motors -> Wing Tips
class ForwardEulerIntegrator:
    def __init__(
        self, envConfig: IntegratorConfig, drone: "Drone", logSchema: dict | None = None
    ) -> None:
        # TODO: update global config
        self.dt = envConfig.dt
        self.drone = drone
        self.logSchema = logSchema

    def step_forward_euler(self, debug: bool = False):
        # --- Validate inputs ---
        dt = self.dt
        if not isinstance(dt, float) or dt <= 0:
            raise TypeError("dt must be a non-negative-zero float")

        mass, inertia = self.drone.model.mass, self.drone.model.inertia_tensor

        # ----------------------------------------------------------------------
        # 1. Unpack state
        # ----------------------------------------------------------------------
        pos_I, vel_B, q, omega_B = self.drone.getStateVec()

        # ----------------------------------------------------------------------
        # 2. Compute total forces and moments in body frame
        # ----------------------------------------------------------------------
        F_B, M_B = self.drone._get_total_forces()

        # ----------------------------------------------------------------------
        # 3. Compute accelerations (body frame)
        # ----------------------------------------------------------------------
        a_B = compute_aB(mass, F_B, omega_B, vel_B)
        alpha_B = compute_alphaB(inertia, M_B, omega_B)

        # ----------------------------------------------------------------------
        # 4. Integrate angular velocity
        # ----------------------------------------------------------------------
        omega_B = omega_B + alpha_B * dt

        # ----------------------------------------------------------------------
        # 5. Quaternion update (using UPDATED omega)
        # ----------------------------------------------------------------------
        rate_q = compute_q_rate(q, omega_B)
        q = q.as_np() + rate_q * dt
        q = Quaternion.unpackArray(q)
        # ----------------------------------------------------------------------
        # 6. Integrate linear velocity (body frame)
        # ----------------------------------------------------------------------
        vel_B = vel_B + a_B * dt

        # ----------------------------------------------------------------------
        # 7. Convert velocity to inertial frame
        # ----------------------------------------------------------------------
        vel_I = EarthFixed.from_BodyFixed(vel_B, q)

        # ----------------------------------------------------------------------
        # 8. Integrate position in inertial frame
        # ----------------------------------------------------------------------
        pos_I = pos_I + vel_I * dt

        # ----------------------------------------------------------------------
        # 9. Write back state
        # ----------------------------------------------------------------------
        state = (pos_I, vel_B, q, omega_B, a_B, alpha_B)
        self.drone.updateStateVector(state)

        if debug:
            # TODO: Add __str__ to drone
            print(body)

    def __call__(self, config: IntegratorConfig):
        return self(config)

    def get_log_definition(self):
        if self.logSchema is None:
            # Default Setting
            raise NotImplementedError
            self.logSchema = create_schema(
                "Integrator",
                [],
                # ["roll_setpoint", "pitch_setpoint", "yaw_setpoint", "omega_setpoint"],
                # ["float", "float", "float", "float"],
                # ["rad", "rad", "rad", "rad"],
            )

        return self.logSchema


def compute_aB(
    mass: int | float, F_B: BodyFixed, omega_B: BodyFixed, vel_B: BodyFixed
) -> BodyFixed:
    # --- mass checks ---
    if not isinstance(mass, (int, float)):
        raise TypeError("mass must be an int or float")

    if mass <= 0:
        raise ValueError("mass must be positive")

    # --- F_B checks ---
    if not isinstance(F_B, BodyFixed):
        raise TypeError("F_B must be a BodyFixed vector")

    if F_B.vec.shape != (3, 1):
        raise ValueError("F_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(F_B.vec)):
        raise ValueError("F_B contains non-finite values")

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- vel_B checks ---
    if not isinstance(vel_B, BodyFixed):
        raise TypeError("vel_B must be a BodyFixed vector")

    if vel_B.vec.shape != (3, 1):
        raise ValueError("vel_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(vel_B.vec)):
        raise ValueError("vel_B contains non-finite values")

    # --- compute acceleration ---
    res = F_B * (1.0 / mass) - omega_B * vel_B
    res.flag = "acceleration"
    return res


def compute_alphaB(
    inertia: np.ndarray, M_B: BodyFixed, omega_B: BodyFixed
) -> BodyFixed:
    # --- inertia checks ---
    if not isinstance(inertia, np.ndarray):
        raise TypeError("inertia must be a numpy ndarray")

    if inertia.shape != (3, 3):
        raise ValueError("inertia must be a 3×3 matrix")

    if not np.all(np.isfinite(inertia)):
        raise ValueError("inertia contains non-finite values")

    if np.linalg.det(inertia) == 0:
        raise ValueError("inertia matrix must be invertible")

    # --- M_B checks ---
    if not isinstance(M_B, BodyFixed):
        raise TypeError("M_B must be a BodyFixed vector")

    if M_B.vec.shape != (3, 1):
        raise ValueError("M_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(M_B.vec)):
        raise ValueError("M_B contains non-finite values")

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- compute angular acceleration ---
    rhs = M_B.vec - np.cross(omega_B.vec, inertia @ omega_B.vec, axis=0)
    alpha_vec = np.linalg.solve(inertia, rhs)

    return BodyFixed.from_Array(alpha_vec, flag="ang_acceleration")


def compute_q_rate(quaternion: Quaternion, omega_B: BodyFixed) -> np.ndarray:
    # --- quaternion checks ---
    quaternion = quaternion.normalized()

    # --- omega_B checks ---
    if not isinstance(omega_B, BodyFixed):
        raise TypeError("omega_B must be a BodyFixed vector")

    if omega_B.vec.shape != (3, 1):
        raise ValueError("omega_B.vec must have shape (3, 1)")

    if not np.all(np.isfinite(omega_B.vec)):
        raise ValueError("omega_B contains non-finite values")

    # --- compute quaternion rate ---
    p, q, r = omega_B.vec.T.ravel()

    mat = np.array(
        [
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0],
        ],
        dtype=float,
    )

    return 0.5 * (mat @ quaternion.as_np())
