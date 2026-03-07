from abc import ABC, abstractmethod
from typing import Tuple

from quad_sim.references.bodyFixed import BodyFixed
from quad_sim.bases.state            import      StateVector
from quad_sim.registry import _register

class EnvironmentEffect(ABC):

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if not getattr(cls, "__abstractmethods__", None):
            _register("effect", cls)

    @abstractmethod
    def apply(self, state:StateVector) -> Tuple[BodyFixed, BodyFixed]:
        """
        Applies the environmental effect to the given state vector
         and returns the resulting forces and moments as a tuple of BodyFixed objects.
        """

class EnvironmentBase(ABC):

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if not getattr(cls, "__abstractmethods__", None):
            _register("environment", cls)

    def __init__(self, effects: list[EnvironmentEffect]):
        self.effects = effects
    
    def apply_effects(self, state:StateVector) -> Tuple[BodyFixed, BodyFixed]:
        """
        Applies all environmental effects to the given state vector and returns the cumulative forces and moments as a tuple of BodyFixed objects.
        """
        total_force = BodyFixed(0, 0, 0)
        total_moment = BodyFixed(0, 0, 0)

        for effect in self.effects:
            force, moment = effect.apply(state)
            total_force += force
            total_moment += moment
        
        return total_force, total_moment