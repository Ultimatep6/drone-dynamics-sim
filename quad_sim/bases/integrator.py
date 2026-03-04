from abc import ABCMeta, abstractmethod, ABC

from quad_sim.templates.droneModel import DroneModel
from quad_sim.dynamics.state import StateVector

class IntegratorBase(ABC):
    def __init__(self, drone_model: DroneModel, dt: float):
        """
        Standard configuration for all integrators.
        
        :param drone_model: Reference to the Drone object (to read/write state).
        :type drone_model: DroneModel
        :param dt: Fixed delta time step.
        :type dt: float
        """

        self.drone = drone_model
        self.dt = dt

        # Basic validation to ensure config is sane
        if self.dt <= 0:
            raise ValueError(f"Time step must be positive, got {self.dt}")

    @abstractmethod
    def step(self, state:StateVector) -> None:
        """
        Function should step forward one set of calculations for the integrator

        :param state: Reference to the StateVector.
        :type state: StateVector
        
        :return: Updates the state of the drone
        """