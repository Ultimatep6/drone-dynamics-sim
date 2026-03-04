from abc import ABC, abstractmethod
from typing import List
import numpy as np

from quad_sim.bases.rigidbody import RigidBody
from quad_sim.bases.motor import MotorBase
from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.dynamics.state import StateVector



class DynamicsBase(ABC):
    def __init_subclass__(cls, **kwargs):
        required_attrs = {'mass', 'inertia_tensor', 'BODY', 'MOTORS'}
        for attr in required_attrs:
            if not hasattr(cls, attr):
                raise NotImplementedError(f"Class {cls.__name__} must define {attr}")
        
        if not isinstance(cls.mass, float):
            raise TypeError("mass must be a float giving the mass of the drone in kg.")
        if not isinstance(cls.inertia_tensor, np.ndarray) or cls.inertia_tensor.shape != (3, 3):
            raise TypeError("inertia_tensor must be a 3x3 numpy array representing the drone's inertia matrix.")
        if not isinstance(cls.BODY, RigidBody):
            raise TypeError("BODY must be an instance of RigidBody representing the drone's physical properties.")
        if not isinstance(cls.MOTORS, List[MotorBase]) or not all(isinstance(motor, MotorBase) for motor in cls.MOTORS):
            raise TypeError("MOTORS must be a list of MotorBase instances representing the drone's motors.")
        
    
    @property
    def mass(self) -> float:
        return self.BODY.mass
    
    @property
    def inertia_tensor(self) -> np.ndarray:
        return self.BODY.inertia_tensor
    
    def compute_forces_and_moments(self) -> tuple[BodyFixed, BodyFixed]:
        """
        Compute the total forces and moments acting on the drone based on the current state vector and motor outputs.

        :return: A tuple containing the total force vector and total moment vector acting on the drone.
        :rtype: tuple[np.ndarray, np.ndarray]
        """
        total_force = BodyFixed(0, 0, 0, flag="force")
        total_moment = BodyFixed(0, 0, 0, flag="moment")

        for motor in self.MOTORS:
            thrust, moment = motor.compute_forces()
            total_force += thrust
            total_moment += moment

            total_moment += motor.position * thrust

        return total_force, total_moment
    
