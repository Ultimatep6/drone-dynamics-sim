from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.dynamics.state import StateVector


def _position_of_point(
        r: BodyFixed,
        state: StateVector,
        extras: BodyFixed = BodyFixed(0, 0, 0),
    ) -> EarthFixed:
        """
        Calculate the position of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        return state.position + EarthFixed.from_BodyFixed(
            r + extras, state.quaternion, flag="position"
        )
    
def _velocity_of_point(
        state: StateVector,
        r: BodyFixed,
        extras: BodyFixed = BodyFixed(0, 0, 0, flag="velocity"),
    ) -> BodyFixed:
        """
        Calculate the velocity of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        return state.velocity + (state.omega * r).changeFlag("velocity") + extras

def _acceleration_of_point(
        state: StateVector,
        r: BodyFixed,
        extras=BodyFixed(0, 0, 0, flag="acceleration"),
    ) -> BodyFixed:
        """
        Calculate the acceleration of a point in the Earth-fixed reference frame.

        :param self: The RigidBody instance.
        :param r: The position vector of the point in the body-fixed reference frame.
        :type r: BodyFixed
        :param state: The current state of the rigid body, including position and orientation.
        :type state: StateVector
        :param extras: Additional offset in the body-fixed reference frame (default is zero).
        :type extras: BodyFixed
        :return: The position of the point in the Earth-fixed reference frame.
        :rtype: EarthFixed
        """
        a = (state.alpha * r).changeFlag("acceleration")
        b = (state.alpha * (state.alpha * r)).changeFlag("acceleration")

        return state.acceleration - a + b