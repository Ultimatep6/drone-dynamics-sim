from abc import ABC, abstractmethod

from quad_sim.bases.state                    import      StateVector

class Constraint(ABC):
    @abstractmethod
    def enforce(self, state: StateVector) -> StateVector:
        """
        Enforce the constraint on the given state vector. 
        This method should modify the state vector in-place to satisfy the constraint.
        :param state: The state vector to be modified to satisfy the constraint.
        :type state: StateVector
        """
        pass

class ConstraintBase(ABC):
    def __init__(self, constraints: list[Constraint]):
        self.constraints = constraints

    def enforce_constraints(self, state: StateVector):
        """
        Enforce all constraints on the given state vector. 
        This method should modify the state vector in-place to satisfy all constraints.
        :param state: The state vector to be modified to satisfy the constraints.
        :type state: StateVector
        :return: The modified state vector that satisfies all constraints.
        :rtype: StateVector
        """
        for constraint in self.constraints:
            state = constraint.enforce(state)

        return state


