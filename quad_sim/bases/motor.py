from abc import ABCMeta, abstractmethod, ABC
from typing import List

from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.math.references.bodyFixed import BodyFixed


class MotorBase(ABC):
    """
    Base class for motor models in the drone simulation. This class defines the required attributes and methods that any specific motor model must implement.
    """
    def __init_subclass__(cls, **kwargs):
        required_attrs = {'position'}#'rpm_squared', 'spin_direction'}
        for attr in required_attrs:
            if not hasattr(cls, attr):
                raise NotImplementedError(f"Class {cls.__name__} must define {attr}")

            if not isinstance(cls.position, BodyFixed):
                raise TypeError("position must be a BodyFixed object representing the motor's position in the body-fixed frame.")
            
        
    @abstractmethod
    def compute_forces(self) -> tuple[BodyFixed,BodyFixed]:
        """
        Must calculate the thrust and moments caused by the propellers (acting through the center of the propeller) in the BodyFrame
        
        :return: Returns the thrust and moment vectors
        :rtype: tuple[BodyFixed, BodyFixed]
        """

    @abstractmethod
    def __generate_propeller_tips(self) -> dict[str,BodyFixed]:
        """
        Generate the initial positions of the propeller tips in the body-fixed reference frame.
        This method should be implemented by subclasses to return the coordinates of each propeller tip,
        identified by a string key (e.g., propeller name or index), mapped to a BodyFixed object representing
        its position relative to the motor's body frame.
        
        :return: Tip_ID and its respective position in the BodyFrame
        :rtype: dict[str, BodyFixed]
        """

    @abstractmethod
    def locate_propeller_tips(self) -> dict[str,BodyFixed]:
        """
        Locate the positions of the propeller tips in the body-fixed reference frame.
        This method should be implemented by subclasses to return the coordinates of each propeller tip,
        identified by a string key (e.g., propeller name or index), mapped to a BodyFixed object representing
        its position relative to the motor's body frame.

        :return: A dictionary mapping propeller identifiers to their positions in the body-fixed frame.
        :rtype: dict[str,BodyFixed]
        """    
