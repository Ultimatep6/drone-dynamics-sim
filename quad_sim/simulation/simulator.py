from dataclasses import astuple, dataclass

from quad_sim.config import (
    IntegratorConfig,
    NCopterConfig,
    SimulatorConfig,
    VehicleConfig,
)
from quad_sim.simulation.drone import Drone
from quad_sim.utils.loggerV2 import NCopterLogger
from quad_sim.utils.logTypes.AutoSignal import AutoSignal
from quad_sim.utils.logTypes.MotorSignal import MotorSignal
from quad_sim.utils.logTypes.StateSignal import StateSignal


class nCopterSimulator:
    def __init__(self, config: NCopterConfig, logDir: str = "test.hdf5") -> None:
        if not isinstance(config, NCopterConfig):
            raise TypeError("config must be a NCopterConfig")

        # initialize the simulation with the Simulator Config
        self.setSimConfig(config.env)

        # Each entity has its own logger instance
        self.__logger = NCopterLogger(logDir)
        self.__entities = {veh.vehicleID: Drone(veh) for veh in config.entities}
        for id, ent in self.__entities.items():
            self.__logger.register_drone(ent)

    def run(self):
        # Calculate the controller required torques
        for id, dr in self.__entities.items():
            # step the drone
            dr.step()
            # log the outputs
            self.__logger.step()

    def close(self):
        self.__logger.finalize()

    @property
    def entities(self):
        return self.__entities

    def appendEntity(self, entityConfig: VehicleConfig):
        self.__entities[entityConfig.vehicleID] = Drone(entityConfig)

    def removeEntity(self, entityID: str):
        if entityID not in self.entities.keys():
            raise ValueError(f"{entityID} is not in the existing entity IDs")

    @property
    def sim(self):
        return self.__sim

    def setSimConfig(self, value: SimulatorConfig):
        if not isinstance(value, SimulatorConfig):
            raise TypeError("New simulation config must be a SimulatorConfig class")
        else:
            self.__sim = value
