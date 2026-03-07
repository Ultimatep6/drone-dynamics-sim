from __future__ import annotations

import numpy as np
from typing import List

from quad_sim.bases.configuration import BuildableConfig
from quad_sim.bases.rigidbody import RigidBody
from quad_sim.bases.state import StateVector
from quad_sim.references.bodyFixed import BodyFixed
from quad_sim.utils.decorators import topLevel

from classes import (
    DefaultAllocator,
    DefaultController,
    DefaultDynamics,
    DefaultEnvironment,
    DefaultIntegrator,
    DefaultMotor,
    DefaultPilot,
    DefaultDrone,
    WindEffect,
)


# ── Motor ────────────────────────────────────────────────────────────────────


class MotorConfig(BuildableConfig):
    """Configuration for a single motor."""

    def __init__(
        self,
        motor_id: str="motor_1",
        spin_direction: int=1,
        position: list[float]=[0.0, 0.0, 0.0],
    ):
        super().__init__(DefaultMotor)
        self.motor_id = motor_id
        self.spin_direction = spin_direction
        self.position = position

    def construct(self) -> DefaultMotor:
        return DefaultMotor(
            id=self.motor_id,
            spin_direction=self.spin_direction,
            position=self.position,
        )


# ── Dynamics ─────────────────────────────────────────────────────────────────


class DynamicsConfig(BuildableConfig):
    """Configuration for the drone dynamics (rigid body + motors)."""

    def __init__(
        self,
        mass: float = 1.0,
        inertia_tensor: np.ndarray = np.ones((3, 3)),
        motors: List[MotorConfig]=[MotorConfig(motor_id=f"motor_{i}", position=BodyFixed(0.1 * i, 0.0, 0.0)) for i in range(4)],
    ):
        super().__init__(DefaultDynamics)
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.motors = motors

    def construct(self) -> DefaultDynamics:
        return DefaultDynamics(
            body=RigidBody(mass=self.mass, inertia_tensor=self.inertia_tensor),
            motors=[m.construct() for m in self.motors],
        )


# ── Pilot ────────────────────────────────────────────────────────────────────


class PilotConfig(BuildableConfig):
    """Configuration for the pilot / autopilot."""

    def __init__(self):
        super().__init__(DefaultPilot)

    def construct(self) -> DefaultPilot:
        return DefaultPilot()


# ── Allocator ────────────────────────────────────────────────────────────────


class AllocatorConfig(BuildableConfig):
    """Configuration for the control allocator."""

    def __init__(self):
        super().__init__(DefaultAllocator)

    def construct(self) -> DefaultAllocator:
        return DefaultAllocator()


# ── Controller ───────────────────────────────────────────────────────────────


class ControllerConfig(BuildableConfig):
    """Configuration for the radio / input controller."""

    def __init__(self):
        super().__init__(DefaultController)

    def construct(self) -> DefaultController:
        return DefaultController()


# ── Integrator ───────────────────────────────────────────────────────────────


class IntegratorConfig(BuildableConfig):
    """Configuration for the numerical integrator."""

    def __init__(self, dt: float=0.1):
        super().__init__(DefaultIntegrator)
        self.dt = dt

    def construct(self) -> DefaultIntegrator:
        return DefaultIntegrator(dt=self.dt)


# ── Environment ──────────────────────────────────────────────────────────────


class EnvironmentConfig(BuildableConfig):
    """Configuration for the simulation environment and its effects."""

    def __init__(self, effects: list | None = [WindEffect()]):
        super().__init__(DefaultEnvironment)
        self.effects = effects

    def construct(self) -> DefaultEnvironment:
        if self.effects is None:
            return DefaultEnvironment()
        return DefaultEnvironment(effects=self.effects)


# ── Drone ────────────────────────────────────────────────────────────────────

@topLevel()
class DroneConfig(BuildableConfig):
    """
    Top-level configuration that composes every subsystem config
    and constructs a fully-assembled DefaultDrone.
    """

    def __init__(
        self,
        drone_id: str,
        init_state: StateVector | None = StateVector(),
        pilot: PilotConfig = PilotConfig(),
        dynamics: DynamicsConfig = DynamicsConfig(),
        allocator: AllocatorConfig = AllocatorConfig(),
        controller: ControllerConfig = ControllerConfig(),
        integrator: IntegratorConfig = IntegratorConfig(dt=0.01),
        environment: EnvironmentConfig = EnvironmentConfig(),
    ):
        super().__init__(DefaultDrone)
        self.drone_id = drone_id
        self.init_state = init_state
        self.pilot = pilot
        self.dynamics = dynamics
        self.allocator = allocator
        self.controller = controller
        self.integrator = integrator
        self.environment = environment

    def construct(self) -> DefaultDrone:
        if self.dynamics is None:
            raise ValueError(
                "DynamicsConfig must be provided before constructing a drone."
            )

        return DefaultDrone(
            drone_id=self.drone_id,
            init_state=self.init_state if self.init_state is not None else StateVector(),
            pilot=self.pilot.construct(),
            dynamics=self.dynamics.construct(),
            allocator=self.allocator.construct(),
            controller=self.controller.construct(),
            integrator=self.integrator.construct(),
            environment=self.environment.construct(),
        )
