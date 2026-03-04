from abc import ABC, ABCMeta, abstractmethod
from typing import List
from dataclasses import dataclass
from enum import Enum, auto

class DesaturationPriority(Enum):
    """
    Defines the priority for the motor allocator when desaturating.
    """
    ATTITUDE = auto()  
    THRUST = auto()    

@dataclass
class FlightModeConfig:
    enable_x_position: bool
    enable_y_position: bool
    enable_z_position: bool
    enable_x_velocity: bool
    enable_y_velocity: bool
    enable_z_velocity: bool

    priority: DesaturationPriority

class FlightModeBase(ABC):
    """
    Base class for flight modes in the drone simulation. 
    This class defines the required attributes and methods that any specific flight mode must implement.
    """

    def __init_subclass__(cls, **kwargs):
        required_attrs = {'config'}
        for attr in required_attrs:
            if not hasattr(cls, attr):
                raise NotImplementedError(f"Class {cls.__name__} must define {attr}")
            if not isinstance(cls.config, FlightModeConfig):
                raise TypeError(f"{cls.__name__}.config must be an instance of FlightModeConfig")
        
    @property
    def desaturation_priority(self) -> FlightModeConfig:
        return self.config.priority
    
    @abstractmethod
    def compute_setpoints(self, control_inputs: dict) -> dict:
        """
        Converts the control inputs into setpoints using the current FlightModeBase.

        This method processes the control inputs and generates the desired setpoints
        for the drone's operation, such as target positions, velocities, or orientations.

        :param control_inputs: A dictionary containing the control inputs from the pilot or autopilot.
        :type control_inputs: dict
        :return: A dictionary containing the computed setpoints based on the flight mode configuration.
        :rtype: dict
        """
        pass
    
    