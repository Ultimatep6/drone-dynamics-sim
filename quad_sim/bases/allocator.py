from abc import ABC, abstractmethod
from typing import Tuple
from quad_sim.math.references.bodyFixed         import      BodyFixed


class AllocatorBase(ABC):

    @abstractmethod
    def allocate(self, thrust_torques: Tuple[BodyFixed, BodyFixed]) -> list[float]:
        """
        Calculate the motor RPMs required to achieve the desired thrust and torques.

        :param thrust_torques: A tuple containing desired thrust and torque values in the body-fixed frame.
        :type thrust_torques: Tuple[BodyFixed, BodyFixed]
        :return: A list of RPM values for each motor.
        :rtype: list[float]
        """
        pass

    

    