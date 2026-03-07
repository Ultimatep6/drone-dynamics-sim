import h5py
import pytest

from exampleSetup.default.config import VehicleConfig
from quad_sim.simulation.drone import Drone
from quad_sim.utils.loggerV2 import NCopterLogger


def test_logger_initial_registration(tmp_path):
    # --- Arrange ---

    filepath = tmp_path / "test_sim.h5"
    logger = NCopterLogger(filepath=str(filepath), chunk_size=8)

    conf = VehicleConfig(vehicleID="drone_A")
    drone = Drone(conf)

    # --- Act ---
    logger.register_drone(drone)

    # --- Assert: registry structure ---
    assert "drone_A" in logger.registry
    assert set(logger.registry["drone_A"].keys()) == {
        "Allocator",
        "Integrator",
        "Controller",
        "State",
    }

    # --- Assert: HDF5 structure exists ---
    with h5py.File(filepath, "r") as f:
        assert "simulation" in f
        assert "drones" in f["simulation"]
        assert "drone_A" in f["simulation/drones"]

        drone_group = f["simulation/drones/drone_A"]

        # Subsystem groups should exist
        assert "Controller" in drone_group
        assert "Allocator" in drone_group

        # But no datasets yet (lazy creation)
        assert len(drone_group["Controller"].keys()) == 0
        assert len(drone_group["Allocator"].keys()) == 0
