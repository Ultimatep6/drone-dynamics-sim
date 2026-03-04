from abc import ABC, ABCMeta, abstractmethod
from typing import List
from quad_sim.math.references.bodyFixed         import      BodyFixed


class AllocatorBase:
    @abstractmethod
    def allocate(self, thrust_torques: List[BodyFixed, BodyFixed]) -> list[float]:
        """
        Compute the motor throttle values required to achieve the desired thrust and torques.

        :param thrust_torques: A list containing desired thrust and torque values in the body-fixed frame.
        :type thrust_torques: List[BodyFixed, BodyFixed]
        :return: A list of throttle values for each motor, where each value is typically normalized between 0 and 1.
        :rtype: list[float]
        """
        pass

    