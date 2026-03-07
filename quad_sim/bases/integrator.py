from abc import abstractmethod, ABC

from quad_sim.bases.state import StateVector
from quad_sim.bases.dynamics import DynamicsBase,RigidBody
from quad_sim.bases.environment import EnvironmentBase
from quad_sim.references.bodyFixed import BodyFixed

class IntegratorBase(ABC):
    def __init__(self, dt: float):
        """
        Standard configuration for all integrators.
        
        :param dt: Fixed delta time step.
        :type dt: float
        """
        self.dt = dt

        # Basic validation to ensure config is sane
        if self.dt <= 0:
            raise ValueError(f"Time step must be positive, got {self.dt}")
        
    @abstractmethod
    def integrate(self, acc:BodyFixed, alpha:BodyFixed, state:StateVector,) -> StateVector:
        """
        Function should step forward one set of calculations for the integrator.
        This is where the intrgration scheme can be implemented (e.g. Euler, RK4, etc.)

        :param state: Reference to the StateVector.
        :type state: StateVector
        
        :return: Returns the updated state of the drone after applying the integration step.
        :rtype: StateVector
        """
        pass

    def step(self, state0:StateVector, model:DynamicsBase, environment: EnvironmentBase) -> StateVector:
        """
        Function should step forward one set of calculations for the integrator

        :param state: Reference to the StateVector.
        :type state: StateVector
        
        :return: Updates the state of the drone
        """

        F, M = model.compute_forces_and_moments(environment, state0)
        a, alpha = model.compute_accelerations(F, M, state0)
        # Update the state vector based on the computed forces and moments
        state1 = self.integrate(a,alpha,state0)

        return state1