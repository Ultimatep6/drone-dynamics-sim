"""
classes_v3.py  –  Concrete implementations (auto-registered)
============================================================

Each class auto-registers itself into the builder's registry
via ``__init_subclass__`` in the ABCs — no decorators needed.
The user imports this module once to trigger class definition
(and thus registration), then calls ``load_drone()``.

    import classes_v3                           # registers everything
    from quad_sim.toml import load_drone
    drone = load_drone("drone_config.toml")     # resolves by name
"""

from __future__ import annotations

import numpy as np
from typing import List, Tuple

from quad_sim.bases.allocator import AllocatorBase, BodyFixed
from quad_sim.bases.controller import ControllerBase
from quad_sim.bases.dynamics import DynamicsBase, RigidBody, MotorBase
from quad_sim.bases.environment import EnvironmentBase, EnvironmentEffect
from quad_sim.bases.pilot import PilotBase
from quad_sim.bases.drone import DroneBase, StateVector, Setpoints
from quad_sim.bases.integrator import IntegratorBase
from quad_sim.bases.constraint import (
    StateConstraint,
    SetpointConstraint,
    ConstraintBase,
)
from quad_sim.orientation.quaternion import Quaternion


# ── Allocator ─────────────────────────────────────────────────────────────

class DefaultAllocator(AllocatorBase):
    def allocate(self, thrust_torques: Tuple[BodyFixed, BodyFixed]) -> list[float]:
        desired_thrust, desired_torque = thrust_torques
        return [0.0, 0.0, 0.0, 0.0]


# ── Controller ─────────────────────────────────────────────────────────────

class DefaultController(ControllerBase):
    def __init__(self):
        self.__connected = True
        self.channels = {
            "throttle": 0.0,
            "roll_angle": 0.0,
            "pitch_angle": 0.0,
            "yaw_angle": 0.0,
        }
        self.switches = {"mode_switch": False}

    def connect(self) -> bool:
        return True

    def calibrate(self, min, max, trim, offset) -> bool:
        return True

    def get_axis_value(self, channel_id: str | List[str]) -> float | dict:
        if isinstance(channel_id, str):
            return self.channels.get(channel_id, 0.0)
        return {ch: self.channels.get(ch, 0.0) for ch in channel_id}

    def get_switch_value(self, switch_id: str | List[str]) -> float | dict:
        if isinstance(switch_id, str):
            return self.switches.get(switch_id, False)
        return {sw: self.switches.get(sw, False) for sw in switch_id}

    def is_connected(self) -> bool:
        return self.__connected is True


# ── Motors ─────────────────────────────────────────────────────────────

class DefaultMotor(MotorBase):
    def __init__(self, id, spin_direction, position, kf=1e-6, km=1e-7,
                 prop_length=0.1, n_props=2):
        super().__init__(id, spin_direction, position)
        self.rpm = 0.0
        self.theta = 0.0
        self.kf = kf
        self.km = km
        self.propTips = self._generate_propeller_tips(prop_length, n_props)

    def iD(self):
        return self._iD

    def compute_forces(self):
        thrust = BodyFixed(0.0, 0.0, self.kf * self.rpm ** 2)
        torque = BodyFixed(0.0, 0.0, -self.spin_direction * self.km * self.rpm ** 2)
        return thrust, torque

    def set_rpm(self, rpm):
        self.rpm = rpm

    def _generate_propeller_tips(self, prop_length=0.1, n_props=2):
        tips = {}
        for i in range(n_props):
            angle = (2 * np.pi / n_props) * i
            tips[f"prop_{i}"] = BodyFixed(
                self.position.x + prop_length * np.cos(angle),
                self.position.y + prop_length * np.sin(angle),
                self.position.z,
            )
        return tips

    def locate_propeller_tips(self):
        return self.propTips

    def update_theta(self, dt):
        return self.theta + (self.rpm / 60.0) * 2 * np.pi * dt


# ── Dynamics ─────────────────────────────────────────────────────────────

class DefaultDynamics(DynamicsBase):
    def __init__(self, body, motors):
        super().__init__(body, motors)

    @property
    def rotor_rates(self):
        return {f"Motor_{i}": m.rpm for i, m in enumerate(self.motors)}


# ── Pilot ─────────────────────────────────────────────────────────────

class DefaultPilot(PilotBase):
    def compute_control(self, state, target):
        return BodyFixed(0.0, 0.0, 0.0), BodyFixed(0.0, 0.0, 0.0)


# ── Integrator ─────────────────────────────────────────────────────────

class DefaultIntegrator(IntegratorBase):
    def __init__(self, dt):
        super().__init__(dt)

    def integrate(self, acc, alpha, q_rate, state):
        return StateVector(
            position=state.position + state.velocity * self.dt,
            velocity=state.velocity + acc * self.dt,
            quaternion=(state.quaternion + q_rate * self.dt).normalized,
            omega=state.omega + alpha * self.dt,
        )


# ── Environment ─────────────────────────────────────────────────────────

class WindEffect(EnvironmentEffect):
    def __init__(self, dir=BodyFixed(1, 0, 0), magnitude=0.0):
        if np.linalg.norm(dir.vec) != 1:
            raise ValueError("Wind direction vector must be a unit vector.")
        self.force = dir * magnitude

    def apply(self, state):
        return self.force, BodyFixed(0, 0, 0)


class DefaultEnvironment(EnvironmentBase):
    def __init__(self, effects=None):
        super().__init__(effects if effects is not None else [WindEffect()])


# ── Constraints ──────────────────────────────────────────────────────────

class GroundPlaneConstraint(SetpointConstraint):
    """Prevents setpoints from requesting an altitude below the floor."""
    def __init__(self, floor: float = 0.0):
        self.floor = floor

    def enforce(self, setpoints: Setpoints) -> Setpoints:
        if setpoints.z is not None and setpoints.z < self.floor:
            setpoints.z = self.floor
        return setpoints


class MaxVelocityConstraint(StateConstraint):
    """Clamps the linear velocity magnitude to a maximum value."""
    def __init__(self, max_speed: float = 30.0):
        self.max_speed = max_speed

    def enforce(self, state: StateVector) -> StateVector:
        speed = float(np.linalg.norm(state.velocity.vec))
        if speed > self.max_speed:
            state.velocity.vec[:] = state.velocity.vec * (self.max_speed / speed)
        return state


class QuaternionNormConstraint(StateConstraint):
    """Re-normalises the orientation quaternion to unit length."""
    def enforce(self, state: StateVector) -> StateVector:
        state.quaternion = state.quaternion.normalized()
        return state


class DefaultConstraints(ConstraintBase):
    """Ships with a sensible default constraint set."""
    def __init__(self, constraints=None):
        if constraints is None:
            constraints = [
                GroundPlaneConstraint(),
                MaxVelocityConstraint(),
            ]
        super().__init__(constraints)


# ── Drone ────────────────────────────────────────────────────────────────

class DefaultDrone(DroneBase):
    def __init__(self, drone_id, init_state, pilot, dynamics, allocator,
                 controller, integrator, environment, constraints):
        super().__init__(drone_id, init_state, pilot, dynamics, allocator,
                         controller, integrator, environment, constraints)

    def get_setpoints(self) -> Setpoints:
        for iD in self.controller.channels:
            self.channels[iD] = self.controller.get_axis_value(iD)
        for iD in self.controller.switches:
            self.switches[iD] = self.controller.get_switch_value(iD)
        return Setpoints(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
