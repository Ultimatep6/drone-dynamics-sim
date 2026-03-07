"""
main_v4.py  –  Fully modular, TOML-driven simulation
======================================================

The builder knows NOTHING about DefaultMotor, DefaultDrone, etc.
It resolves them by name from the registry at build-time.

Step 1: import your classes module (triggers __init_subclass__ auto-registration)
Step 2: load_drone("file.toml")
"""

# Step 1 — importing this module defines all the Default* classes,
#           which auto-register via __init_subclass__ in the ABCs.
#           A different project would import *its own* classes module instead.
import classes_v3  # noqa: F401  (side-effect: populates the registry)

from quad_sim.toml import load_drone
from sim_v2 import Simulator


# ═══════════════════════════════════════════════════════════════════════════
# 1.  Minimal — all defaults
# ═══════════════════════════════════════════════════════════════════════════

simple = load_drone("minimal.toml")


# ═══════════════════════════════════════════════════════════════════════════
# 2.  Full config
# ═══════════════════════════════════════════════════════════════════════════

betty = load_drone("drone_config.toml")


# ═══════════════════════════════════════════════════════════════════════════
# 3.  Same TOML, runtime overrides (parameter sweeps)
# ═══════════════════════════════════════════════════════════════════════════

heavy = load_drone("drone_config.toml", overrides={"body.mass": 3.0})


# ═══════════════════════════════════════════════════════════════════════════
# 4.  Run
# ═══════════════════════════════════════════════════════════════════════════

sim = Simulator([simple, betty, heavy])
print(sim)

for _ in range(100):
    sim.run()

print(f"\nBetty: mass={betty.model.mass}, dt={betty.integrator.dt}")
print(f"Heavy: mass={heavy.model.mass}, dt={heavy.integrator.dt}")
