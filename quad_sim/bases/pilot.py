from abc import ABCMeta, abstractmethod, ABC
from typing import List

from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.dynamics.state            import      StateVector


class PilotBase(ABC):
    """
    The Pilot ABC shall do:
        - Receive the inputs of an RController
    """
    @abstractmethod
    def step_PID(self, state:StateVector) -> List[BodyFixed, BodyFixed]:
        """
        Returns the thrust and torques in the Body Frame.

        This method calculates and returns the thrust and torques required for the drone's operation
        in the body-fixed reference frame.

        Returns:
            List[BodyFixed, BodyFixed]: A list containing thrust and torques in the body frame.
        """
        pass
    