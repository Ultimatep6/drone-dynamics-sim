from abc import ABC, abstractmethod
from typing import List

class ControllerBase(ABC):
    """
    Abstract base class for a radio controller interface in a drone simulation.
    Defines the required methods for connecting, calibrating, reading axis and switch values, and checking connection status.
    Implementations should provide hardware or software-specific logic for these operations.
    """
    @abstractmethod
    def connect(self) -> bool:
        """
        Connecting the controller hardware/software to the simulation.
        
        :return: Boolean confirmation of a connection.
        :rtype: bool
        """
        
        pass

    @abstractmethod
    def calibrate(self, min:int|dict, max:int|dict, trim:int|dict, offset:int|dict) -> bool:
        """
        Calibrates the input ranges of the RadioController to the simulation ranges.
        Stores the hardware ranges internally.
        Apply offset to avoid stick drift.
        
        :param min: Contains the minimum controller-equivalent inputs
        :type min: int | dict
        :param max: Contains the maximum controller-equivalent inputs
        :type max: int | dict
        :param trim: Contains the trim point (center) of controller-equivalent inputs
        :type trim: int | dict
        :param offset: Contains the range for the 'dead-zone' of the controller-equivalent inputs
        :type offset: int | dict
        :return: Boolean verification of successful calibration
        :rtype: bool
        """

        pass

    @abstractmethod
    def get_axis_value(self,channel_id:str|List[str]) -> float|dict:
        """
        Using the calibrated calculation formula, returns the simulation equivalent input of a controller input
        
        :param channel_id: Identifier for a given input joystick/pad interface
        :type channel_id: str|List[str]
        :return: Simulation-translated input from the controller
        :rtype: float|dict
        """
        pass

    @abstractmethod
    def get_switch_value(self,switch_id:str|List[str]) -> float|dict:
        """
        Returns the simulation equivalent input of a controller boolean switch
        
        :param switch_id: Identifier for a given input boolean interface
        :type switch_id: str|List[str]
        :return: Simulation-translated input from the controller
        :rtype: float|dict
        """

        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if the controler is still connected to simulation.
        Should include some fail safes
        
        :return: Boolean verification that the connection is successful or not
        :rtype: bool
        """


