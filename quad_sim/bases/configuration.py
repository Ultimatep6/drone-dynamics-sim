from abc import ABC, abstractmethod
from quad_sim.bases.drone import DroneBase
from quad_sim.bases.flightmode import FlightModeBase
from quad_sim.bases.dynamics import DynamicsBase
from quad_sim.bases.pilot import PilotBase
from quad_sim.bases.controller import ControllerBase
from quad_sim.bases.allocator import AllocatorBase
from quad_sim.bases.integrator import IntegratorBase
from quad_sim.bases.environment import EnvironmentBase


BASES = [
    DroneBase,
    FlightModeBase,
    DynamicsBase,
    PilotBase,
    ControllerBase,
    AllocatorBase,
    IntegratorBase,
    EnvironmentBase
]

class BuildableConfig(ABC):
    """
    A high level configuration setup that allows itself and its children config to be built as one of the base model object
    """
    @abstractmethod
    def __init__(self, constructObj: type):
        """
        Should contain the basics that the configuration can use to construct the base model object
        
        :param construct: The base model class
        :type construct: type
        """
        if not issubclass(constructObj, ABC):
            raise TypeError("constructObj must be a subclass of a baseModel")
        self._construct = constructObj
    
    @abstractmethod
    def construct(self) -> ABC:
        """
        Constructs and returns an instance of the base model object
        (e.g. DroneBase, DynamicsBase, PilotBase, etc.).

        :return: A fully constructed instance of the base class that was
                 registered via ``constructObj`` in ``__init__``.
        :rtype: An instance of one of the registered base classes.
        """
        pass

