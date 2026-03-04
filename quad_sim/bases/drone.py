from abc import ABC, ABCMeta, abstractmethod
from typing import List

from quad_sim.bases.dynamics            import      DynamicsBase
from quad_sim.bases.controller          import      ControllerBase
from quad_sim.bases.pilot               import      PilotBase
from quad_sim.bases.allocator           import      AllocatorBase
from quad_sim.bases.integrator           import     IntegratorBase


from quad_sim.config                            import      VehicleConfig
from quad_sim.dynamics.state                    import      StateVector
from quad_sim.math.references.bodyFixed         import      BodyFixed



class DroneBase(ABC):

    def __init_subclass__(cls, **kwargs):
        required_attrs = {'iD', 'PILOT', 'ALLOCATOR', 'MODEL', 'CONTROLLER', 'INTEGRATOR'}
        for attr in required_attrs:
            if not hasattr(cls, attr):
                raise NotImplementedError(f"Class {cls.__name__} must define {attr}")
        
        if not isinstance(cls.iD, str):
            raise TypeError(f"{cls.__name__}.iD must be a string identifier for the drone model.")
        
        if not issubclass(cls.PILOT, PilotBase):
            raise TypeError(f"{cls.__name__}.PILOT must be a subclass of PilotBase")
        
        if not issubclass(cls.ALLOCATOR, AllocatorBase):
            raise TypeError(f"{cls.__name__}.ALLOCATOR must be a subclass of AllocatorBase")
        
        if not issubclass(cls.MODEL, DynamicsBase):
            raise TypeError(f"{cls.__name__}.MODEL must be a subclass of DynamicsBase")
        
        if not issubclass(cls.CONTROLLER, ControllerBase):
            raise TypeError(f"{cls.__name__}.CONTROLLER must be a subclass of ControllerBase")

        if not issubclass(cls.INTEGRATOR, IntegratorBase):
            raise TypeError(f"{cls.__name__}.INTEGRATOR must be a subclass of IntegratorBase")

    def step(self) -> None:
        """
        Advances the simulation by one time step.
        This method updates the state of the drone model, applying control inputs,
        updating physics, and recalculating sensor readings as necessary.
        """
        self.target = self.get_setpoints()
        self.response = self.step_PID(self.target)
        self.setThrottle(self.ALLOCATOR.calcRates(self.response))
        
        self.INTEGRATOR.step()

        pass

    @abstractmethod
    def get_state(self) -> dict:
        """
        Returns the current state of the drone.
        This method retrieves and returns the current state variables of the drone,
        such as position, velocity, orientation, and angular velocity.
        Returns:
            dict: A dictionary containing the current state of the drone with keys such as
                  'position', 'velocity', 'orientation', and 'angular_velocity'.
        """
        pass

    @abstractmethod
    def get_setpoints(self) -> dict:
        """
        Converts the control inputs into setpoints using the current FlightModeBase.

        This method processes the control inputs and generates the desired setpoints
        for the drone's operation, such as target positions, velocities, or orientations.

        Returns:
            dict: A dictionary containing the setpoints with keys such as
                  'position_setpoint', 'velocity_setpoint', 'orientation_setpoint', 'thrust_setpoint'.
        """
        pass

    

        