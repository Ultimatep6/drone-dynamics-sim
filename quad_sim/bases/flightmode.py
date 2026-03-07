from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto

from quad_sim.bases.setpoints import Setpoints
from quad_sim.registry import _register


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


class FlightModes:
    """
    Dynamic catalog of all registered flight modes.

    Every concrete ``FlightModeBase`` subclass is automatically added here
    as a class attribute under its own name.  This lets you reference
    modes like an enum::

        FlightModes.Stabilise          # → <class 'Stabilise'>
        FlightModes.AltitudeHold       # → <class 'AltitudeHold'>
        FlightModes.available()        # → ['Stabilise', 'AltitudeHold', ...]

    Use ``FlightModes.get("Stabilise")`` to look up by string (e.g. from
    a TOML file or controller switch).
    """

    @classmethod
    def _add(cls, mode_cls: type) -> None:
        """Register a concrete FlightModeBase subclass."""
        setattr(cls, mode_cls.__name__, mode_cls)

    @classmethod
    def available(cls) -> list[str]:
        """Return the names of all registered flight modes."""
        return [
            name for name, val in vars(cls).items()
            if isinstance(val, type) and issubclass(val, FlightModeBase)
        ]

    @classmethod
    def get(cls, name: str) -> type:
        """
        Look up a flight mode class by name.

        :param name: The class name of the flight mode.
        :type name: str
        :raises KeyError: If no flight mode with that name is registered.
        :return: The flight mode class.
        :rtype: type[FlightModeBase]
        """
        mode = getattr(cls, name, None)
        if mode is None or not (isinstance(mode, type) and issubclass(mode, FlightModeBase)):
            raise KeyError(
                f"Unknown flight mode '{name}'. "
                f"Available: {cls.available()}"
            )
        return mode


class FlightModeBase(ABC):
    """
    Base class for flight modes in the drone simulation. 
    This class defines the required attributes and methods that any specific flight mode must implement.

    Concrete subclasses are automatically:
    - registered in the builder registry (slot ``"flight-mode"``)
    - added to :class:`FlightModes` as a named attribute
    """

    def __init__(self, mode: FlightModeConfig):
        if not isinstance(mode, FlightModeConfig):
            raise TypeError(f"mode must be an instance of FlightModeConfig, got {type(mode)}")
        
        self.mode = mode

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if not getattr(cls, "__abstractmethods__", None):
            _register("flight-mode", cls)
            FlightModes._add(cls)

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
    
    