from abc import abstractmethod, ABC
from typing import Tuple

from quad_sim.references.bodyFixed import BodyFixed
from quad_sim.bases.state            import      StateVector
from quad_sim.bases.setpoints import Setpoints
from quad_sim.registry import _register


class PilotBase(ABC):
    """
    The Pilot ABC shall do:
        - Receive the inputs of an RController
    """

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if not getattr(cls, "__abstractmethods__", None):
            _register("pilot", cls)

    @abstractmethod
    def compute_control(self, state:StateVector, setpoints: Setpoints) -> Tuple[BodyFixed, BodyFixed]:
        """
        Computes the control outputs based on the current state and desired setpoints.

        :param state: The current state of the system.
        :type state: StateVector
        :param setpoints: The desired setpoints for the system.
        :type setpoints: Setpoints
        :return: A tuple containing two BodyFixed objects representing the thrust and moments.
        :rtype: Tuple[BodyFixed, BodyFixed]
        """
        pass
    