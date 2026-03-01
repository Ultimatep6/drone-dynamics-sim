import time

from quad_sim.config import (
    NCopterConfig,
    PhysicalLimits,
    SimulatorConfig,
    ThrustConfig,
    VehicleConfig,
    VelocityLimits,
)
from quad_sim.control.config.autopilotConf import AutopilotConfig
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.control.config.gains import GainsConf, PidGains
from quad_sim.simulation.simulator import nCopterSimulator
from quad_sim.viz.reader import plot_comparison, plot_field, print_hdf5_contents

config = NCopterConfig(
    entities=[
        VehicleConfig(
            physical_limits=PhysicalLimits(
                thrust_limits=ThrustConfig(max_thrust=1000),
                velocity_limits=VelocityLimits(max_vz=5.0),
            ),
            pid_config=AutopilotConfig(
                gains=GainsConf(vz=PidGains(kp=0.3, kd=0.18, ki=0.1, alpha=0.85)),
            ),
        )
    ],
    env=SimulatorConfig(),
)

# WORKS
sim = nCopterSimulator(config, "examples/example2/outputs/test.hdf5")
dr = sim.entities["drone"]

dr.setMode(FlightMode.ALTITUDE)

dr.updateRC("Ly", 0.1)

for i in range(400):
    sim.run()

dr.updateRC("Ly", 0.8)

for i in range(300):
    sim.run()

dr.updateRC("Ly", 0.0)

for i in range(300):
    sim.run()

sim.close()

plot_comparison(
    paths=["examples/example2/outputs/test.hdf5"],
    drone="drone",
    subsystems=["Controller", "State"],
    field_a="velocity_setpoint",
    field_b="velocity",
)

plot_field(
    "examples/example2/outputs/test.hdf5", "drone", "Controller", "thrust_setpoint"
)
