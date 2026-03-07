from abc import ABC, abstractmethod

from quad_sim.bases.state                    import      StateVector
from quad_sim.bases.setpoints           import      Setpoints


class Constraint(ABC):
    """Marker base every concrete constraint must subclass either
    StateConstraint or SetpointConstraint, never Constraint directly."""

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)

        # Allow the two intermediate ABCs themselves
        if cls.__name__ in ("StateConstraint", "SetpointConstraint"):
            return

        # Skip further abstract intermediaries
        if getattr(cls, "__abstractmethods__", None):
            return

        is_state = issubclass(cls, StateConstraint)
        is_setpoint = issubclass(cls, SetpointConstraint)

        if is_state and is_setpoint:
            raise TypeError(
                f"{cls.__name__} subclasses both StateConstraint and "
                f"SetpointConstraint. A constraint must be exactly one."
            )
        if not is_state and not is_setpoint:
            raise TypeError(
                f"{cls.__name__} must subclass either "
                f"StateConstraint or SetpointConstraint."
            )


class StateConstraint(Constraint, ABC):
    """A constraint that operates on StateVector."""

    @abstractmethod
    def enforce(self, state: StateVector) -> StateVector:
        """
        Enforce the constraint on the given state vector.
        :param state: The state vector to be modified to satisfy the constraint.
        :type state: StateVector
        :return: The modified state vector.
        :rtype: StateVector
        """
        pass


class SetpointConstraint(Constraint, ABC):
    """A constraint that operates on Setpoints."""

    @abstractmethod
    def enforce(self, setpoints: Setpoints) -> Setpoints:
        """
        Enforce the constraint on the given setpoints.
        :param setpoints: The setpoints to be modified to satisfy the constraint.
        :type setpoints: Setpoints
        :return: The modified setpoints.
        :rtype: Setpoints
        """
        pass


class ConstraintBase(ABC):
    def __init__(self, constraints: list[Constraint]):
        self.state_constraints: list[StateConstraint] = []
        self.setpoint_constraints: list[SetpointConstraint] = []

        for c in constraints:
            if isinstance(c, StateConstraint):
                self.state_constraints.append(c)
            elif isinstance(c, SetpointConstraint):
                self.setpoint_constraints.append(c)
            else:
                # Should never reach here thanks to __init_subclass__,
                # but guard against misuse.
                raise TypeError(
                    f"{c.__class__.__name__} is not a StateConstraint or "
                    f"SetpointConstraint."
                )

    def enforce_state_constraints(self, state: StateVector) -> StateVector:
        """
        Enforce all state constraints on the given state vector.
        :param state: The state vector to be modified to satisfy all constraints.
        :type state: StateVector
        :return: The modified state vector that satisfies all constraints.
        :rtype: StateVector
        """
        for constraint in self.state_constraints:
            state = constraint.enforce(state)
        return state

    def enforce_setpoint_constraints(self, setpoints: Setpoints) -> Setpoints:
        """
        Enforce all setpoint constraints on the given setpoints.
        :param setpoints: The setpoints to be modified to satisfy all constraints.
        :type setpoints: Setpoints
        :return: The modified setpoints that satisfy all constraints.
        :rtype: Setpoints
        """
        for constraint in self.setpoint_constraints:
            setpoints = constraint.enforce(setpoints)
        return setpoints


