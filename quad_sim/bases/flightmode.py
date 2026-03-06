from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto

from quad_sim.bases.setpoints import Setpoints

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

    def __init__(self, mode: FlightModeConfig):
        if not isinstance(mode, FlightModeConfig):
            raise TypeError(f"mode must be an instance of FlightModeConfig, got {type(mode)}")
        
        self.mode = mode

    @property
    def desaturation_priority(self) -> DesaturationPriority:
        return self.mode.priority
    
    @abstractmethod
    def compute_setpoints(self, control_inputs: Setpoints) -> Setpoints:
        """
        Converts the control inputs into setpoints using the current FlightModeBase.

        This method processes the control inputs and generates the desired setpoints
        for the drone's operation, such as target positions, velocities, or orientations.

        :param control_inputs: A Setpoints containing the control inputs from the pilot or autopilot.
        :type control_inputs: Setpoints
        :return: The computed setpoints based on the flight mode configuration.
        :rtype: Setpoints
        """
        pass
    
    