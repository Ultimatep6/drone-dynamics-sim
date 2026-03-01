import time

from quad_sim.config import (
    NCopterConfig,
    PhysicalLimits,
    SimulatorConfig,
    ThrustConfig,
    VehicleConfig,
)
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.simulation.simulator import nCopterSimulator
from quad_sim.viz.reader import plot_field, print_hdf5_contents

config = NCopterConfig(
    entities=[
        VehicleConfig(
            physical_limits=PhysicalLimits(
                thrust_limits=ThrustConfig(max_thrust=9.81 * 2)
            )
        )
    ],
    env=SimulatorConfig(),
)

# WORKS
sim = nCopterSimulator(config, "examples/example1/outputs/test.hdf5")
dr = sim.entities["drone"]


dr.updateRC("Ly", 0.5)

for i in range(10):
    sim.run()

dr.updateRC("Ly", 0)
for i in range(10):
    sim.run()

dr.updateRC("Ly", 1.0)

for i in range(10):
    sim.run()

sim.close()

time.sleep(5)

plot_field("examples/example1/outputs/test.hdf5", "drone", "State", "velocity")
