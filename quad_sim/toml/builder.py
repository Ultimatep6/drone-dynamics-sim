"""
builder.py  –  Generic TOML-driven drone builder
==================================================

This module is completely decoupled from any concrete implementation.
It works only with the ABCs defined in ``quad_sim.bases.*``.

Concrete classes register themselves **automatically** via
``__init_subclass__`` hooks in the ABCs — no decorators needed.
The TOML file references them **by name**.  The builder resolves
names → classes at build-time.

Usage (framework side — just subclass an ABC):

    class MyMotor(MotorBase): ...        # auto-registered as "motor"
    class MyDynamics(DynamicsBase): ...  # auto-registered as "dynamics"
    class MyDrone(DroneBase): ...        # auto-registered as "drone"

Usage (user side):

    from quad_sim.toml.builder import load_drone
    drone = load_drone("my_drone.toml")

The TOML file names the concrete classes it wants:

    [classes]
    drone       = "MyDrone"
    motor       = "MyMotor"
    dynamics    = "MyDynamics"
    pilot       = "MyPilot"
    allocator   = "MyAllocator"
    controller  = "MyController"
    integrator  = "MyIntegrator"
    environment = "MyEnvironment"
    constraints = "MyConstraints"

Omit any key to use the first (or only) registered class for that slot.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import tomllib  # stdlib ≥ 3.11
import numpy as np

from quad_sim.bases.allocator import AllocatorBase
from quad_sim.bases.constraint import Constraint, ConstraintBase
from quad_sim.bases.controller import ControllerBase
from quad_sim.bases.drone import DroneBase
from quad_sim.bases.dynamics import DynamicsBase
from quad_sim.bases.environment import EnvironmentBase, EnvironmentEffect
from quad_sim.bases.integrator import IntegratorBase
from quad_sim.bases.motor import MotorBase
from quad_sim.bases.pilot import PilotBase
from quad_sim.bases.rigidbody import RigidBody
from quad_sim.bases.state import StateVector
from quad_sim.references.bodyFixed import BodyFixed
from quad_sim.references.earthFixed import EarthFixed
from quad_sim.orientation.quaternion import Quaternion
from quad_sim.registry import _resolve


# ═══════════════════════════════════════════════════════════════════════════
#  TOML Loader
# ═══════════════════════════════════════════════════════════════════════════

def load_drone(
    path: str | Path,
    overrides: dict[str, Any] | None = None,
) -> DroneBase:
    """
    Parse a TOML file and return a fully-constructed ``DroneBase`` subclass.

    The concrete classes used are determined by the ``[classes]`` table
    in the TOML.  Every section and every key has a sensible fallback so
    a minimal file can be as short as::

        drone_id = "my_quad"

    Parameters
    ----------
    path : str | Path
        Path to the ``.toml`` configuration file.
    overrides : dict, optional
        Dot-separated key→value pairs applied on top of the parsed file,
        e.g. ``{"body.mass": 2.0, "integrator.dt": 0.001}``.

    Returns
    -------
    DroneBase
        A ready-to-simulate drone instance.
    """
    path = Path(path)
    with path.open("rb") as f:
        cfg = tomllib.load(f)

    # ── apply overrides ──
    if overrides:
        for dotted_key, value in overrides.items():
            parts = dotted_key.split(".")
            target = cfg
            for part in parts[:-1]:
                target = target.setdefault(part, {})
            target[parts[-1]] = value

    # ── resolve class names ──
    classes = cfg.get("classes", {})

    DroneCls:       type[DroneBase]       = _resolve("drone",       classes.get("drone"))
    MotorCls:       type[MotorBase]       = _resolve("motor",       classes.get("motor"))
    DynamicsCls:    type[DynamicsBase]    = _resolve("dynamics",    classes.get("dynamics"))
    PilotCls:       type[PilotBase]       = _resolve("pilot",       classes.get("pilot"))
    AllocatorCls:   type[AllocatorBase]   = _resolve("allocator",   classes.get("allocator"))
    ControllerCls:  type[ControllerBase]  = _resolve("controller",  classes.get("controller"))
    IntegratorCls:  type[IntegratorBase]  = _resolve("integrator",  classes.get("integrator"))
    EnvironmentCls: type[EnvironmentBase] = _resolve("environment", classes.get("environment"))
    ConstraintsCls: type[ConstraintBase]  = _resolve("constraints", classes.get("constraints"))

    # ── identity ──
    drone_id: str = cfg.get("drone_id", "drone_0")

    # ── state ──
    state_cfg = cfg.get("state", {})
    if state_cfg:
        init_state = StateVector(
            position=EarthFixed.from_Array(
                np.array(state_cfg.get("position", [0, 0, 0]), dtype=float).reshape(3, 1)
            ),
            velocity=BodyFixed.from_Array(
                np.array(state_cfg.get("velocity", [0, 0, 0]), dtype=float).reshape(3, 1),
                flag="velocity",
            ),
            quaternion=Quaternion(*state_cfg.get("quaternion", [1, 0, 0, 0])),
            omega=BodyFixed.from_Array(
                np.array(state_cfg.get("omega", [0, 0, 0]), dtype=float).reshape(3, 1),
                flag="ang_velocity",
            ),
        )
    else:
        init_state = StateVector()

    # ── body ──
    body_cfg = cfg.get("body", {})
    mass = body_cfg.get("mass", 1.0)
    inertia = np.array(
        body_cfg.get("inertia", np.eye(3).tolist()), dtype=float
    )

    # ── motors ──
    motor_blocks = cfg.get("motors", _default_motor_blocks())
    motors = []
    for m in motor_blocks:
        # Forward every key except 'type' to the constructor.
        # 'type' can optionally override the motor class per-motor.
        m = dict(m)
        per_motor_cls_name = m.pop("type", None)
        cls = _resolve("motor", per_motor_cls_name) if per_motor_cls_name else MotorCls

        # Map TOML keys → MotorBase.__init__ parameter names
        motors.append(
            cls(
                id=m.get("motor_id", f"motor_{len(motors)}"),
                spin_direction=m.get("spin_direction", 1),
                position=BodyFixed(*m.get("position", [0, 0, 0])),
                **{k: v for k, v in m.items()
                   if k not in ("motor_id", "spin_direction", "position")},
            )
        )

    dynamics = DynamicsCls(
        body=RigidBody(mass=mass, inertia_tensor=inertia),
        motors=motors,
    )

    # ── integrator ──
    int_cfg = cfg.get("integrator", {})
    dt = int_cfg.get("dt", 0.01)
    integrator = IntegratorCls(dt=dt)

    # ── environment effects ──
    effect_blocks = cfg.get("effects", [])
    effects = []
    for e in effect_blocks:
        e = dict(e)
        effect_cls_name = e.pop("type")
        effect_cls = _resolve("effect", effect_cls_name)
        effects.append(effect_cls(**e))

    env_cfg = cfg.get("environment", {})
    if effects:
        environment = EnvironmentCls(effects=effects)
    elif env_cfg:
        # Shorthand: wind_direction + wind_magnitude in [environment]
        # Only works if the EnvironmentCls accepts effects=
        wind_dir = env_cfg.get("wind_direction", [1, 0, 0])
        wind_mag = env_cfg.get("wind_magnitude", 0.0)
        # Resolve a single wind effect from the registry
        wind_cls = _resolve("effect")
        environment = EnvironmentCls(
            effects=[wind_cls(dir=BodyFixed(*wind_dir), magnitude=wind_mag)]
        )
    else:
        environment = EnvironmentCls(effects=[])

    # ── constraints ──
    constraint_blocks = cfg.get("constraints", [])
    constraint_instances: list[Constraint] = []
    for c in constraint_blocks:
        c = dict(c)
        cls_name = c.pop("type")
        cls = _resolve("constraint", cls_name)
        constraint_instances.append(cls(**c))

    constraints = ConstraintsCls(
        constraints=constraint_instances if constraint_instances else None
    )

    # ── pilot / allocator / controller ──
    pilot = PilotCls()
    allocator = AllocatorCls()
    controller = ControllerCls()

    # ── assemble drone ──
    return DroneCls(
        drone_id=drone_id,
        init_state=init_state,
        pilot=pilot,
        dynamics=dynamics,
        allocator=allocator,
        controller=controller,
        integrator=integrator,
        environment=environment,
        constraints=constraints,
    )


# ── helpers ──────────────────────────────────────────────────────────────

def _default_motor_blocks() -> list[dict]:
    """Quad-X layout when the TOML has no [[motors]] section."""
    return [
        {
            "motor_id": f"motor_{i}",
            "spin_direction": 1 if i % 2 == 0 else -1,
            "position": [
                round(0.15 * np.cos(i * np.pi / 2), 6),
                round(0.15 * np.sin(i * np.pi / 2), 6),
                0.0,
            ],
        }
        for i in range(4)
    ]
