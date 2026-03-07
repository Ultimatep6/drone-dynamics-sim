import numpy as np

from quad_sim.bases.allocator import AllocatorBase, BodyFixed, Tuple
from quad_sim.bases.controller import ControllerBase, List
from quad_sim.bases.dynamics import DynamicsBase, RigidBody, MotorBase
from quad_sim.bases.environment import EnvironmentBase
from quad_sim.bases.pilot import PilotBase
from quad_sim.bases.drone import DroneBase, StateVector, Setpoints
from quad_sim.bases.integrator import IntegratorBase
from quad_sim.bases.environment import EnvironmentBase, EnvironmentEffect
from quad_sim.bases.constraint import StateConstraint, SetpointConstraint, ConstraintBase

from quad_sim.orientation.quaternion import from quad_sim.orientation.quaternion import Quaternion



# ── Allocator ─────────────────────────────────────────────────────────────
class DefaultAllocator(AllocatorBase):
    def allocate(self, thrust_torques: Tuple[BodyFixed, BodyFixed]) -> list[float]:
        # Placeholder implementation: This should be replaced with actual allocation logic
        desired_thrust, desired_torque = thrust_torques
        # For demonstration, we return a list of zeros. Replace with actual calculations.
        return [0.0, 0.0, 0.0, 0.0]
    
# ── Controller ─────────────────────────────────────────────────────────────
class DefaultController(ControllerBase):
    def __init__(self):
        self.__connected = True  # Simulate a successful connection for demonstration purposes
        
        self.channels = {'throttle':0.0, 'roll_angle':0.0, 'pitch_angle':0.0, 'yaw_angle':0.0}
        self.switches = {'mode_switch':False}
        

    def connect(self) -> bool:
        # Placeholder implementation: Simulate a successful connection // Always return True for demonstration purposes
        return True
    
    def calibrate(self, min:int|dict, max:int|dict, trim:int|dict, offset:int|dict) -> bool:
        # Placeholder implementation: Simulate successful calibration
        return True
    
    def get_axis_value(self, channel_id:str|List[str]) -> float|dict:
        if not all(isinstance(ch, str) for ch in (channel_id if isinstance(channel_id, list) else [channel_id])):
            raise ValueError("channel_id must be a string or a list of strings representing channel names.")
        if isinstance(channel_id, str):
            return self.channels.get(channel_id, 0.0)
        elif isinstance(channel_id, list):
            return {ch: self.channels.get(ch, 0.0) for ch in channel_id}
        return 0.0
    
    def get_switch_value(self, switch_id:str|List[str]) -> float|dict:
        if not all(isinstance(sw, str) for sw in (switch_id if isinstance(switch_id, list) else [switch_id])):
            raise ValueError("switch_id must be a string or a list of strings representing switch names.")
        if isinstance(switch_id, str):
            return self.switches.get(switch_id, False)
        elif isinstance(switch_id, list):
            return {sw: self.switches.get(sw, False) for sw in switch_id}
        return 0.0
    
    def is_connected(self) -> bool:
        # Placeholder implementation: Simulate that the controller is always connected
        return self.__connected is True
    
# ── Motors ─────────────────────────────────────────────────────────────
class DefaultMotor(MotorBase):
    def __init__(self, id:str, spin_direction: int, position: BodyFixed, kf: float = 1e-6, km: float = 1e-7, propLength: float = 0.1, nProps: int = 2):
        super().__init__(id, spin_direction, position)

        self.spin_direction = spin_direction  # +1 for CCW, -1 for CW (NED-z)
        self.position = position  # Position of the motor in the body-fixed frame

        self.rpm = 0.0  # Initialize rpm to zero
        self.theta = 0.0     # Initialize propeller angle to zero

        self.kf = kf  # Thrust coefficient (placeholder value)
        self.km = km  # Torque coefficient (placeholder value)

        self.propTips = self._generate_propeller_tips(propLength,nProps)  # Generate propeller tip positions based on motor position and propeller length

    def iD(self) -> str:
        return self._iD
    
    def compute_forces(self) -> Tuple[BodyFixed, BodyFixed]:
        # Thrust
        thrust = BodyFixed(0.0, 0.0, self.kf * self.rpm**2)
        # Torque
        torque = BodyFixed(0.0, 0.0, -self.spin_direction * self.km * self.rpm**2)
        return thrust, torque
    
    def set_rpm(self, rpm: float) -> None:
        # Placeholder implementation: Set the throttle based on RPM (this is just a dummy relationship)
        self.rpm = rpm


    def _generate_propeller_tips(self, propLength:float, nProps:int) -> dict[str, BodyFixed]:
        # Generate propeller tip positions based on motor position and propeller length
        propTips = {}
        for i in range(nProps):
            angle = (2 * np.pi / nProps) * i
            tip_x = self.position.x + propLength * np.cos(angle)
            tip_y = self.position.y + propLength * np.sin(angle)
            tip_z = self.position.z  # Assuming the propeller lies in the plane parallel to the body frame
            propTips[f"prop_{i}"] = BodyFixed(tip_x, tip_y, tip_z)
        return propTips
    
    def locate_propeller_tips(self):
        return self._generate_propeller_tips()
    
    def update_theta(self, dt):
        return self.theta + (self.rpm / 60.0) * 2 * np.pi * dt  # Update theta based on RPM and time step

# ── Dynamics ─────────────────────────────────────────────────────────────
class DefaultDynamics(DynamicsBase):
    def __init__(self, body: RigidBody, motors: List[MotorBase]):
        super().__init__(body, motors)
    
    @property
    def rotor_rates(self) -> dict[str, float]:
        # Placeholder implementation: Return default rotor rates
        return {f"Motor_{i}": motor.rpm for i,motor in enumerate(self.motors)}

# ── Pilot ─────────────────────────────────────────────────────────────
class DefaultPilot(PilotBase):
    def compute_control(self, state: StateVector, target: Setpoints) -> Tuple[BodyFixed, BodyFixed]:
        # Placeholder implementation: Return zero thrust and moments
        return BodyFixed(0.0, 0.0, 0.0), BodyFixed(0.0, 0.0, 0.0)
    
# ── Integrator ─────────────────────────────────────────────────────────────
class DefaultIntegrator(IntegratorBase):
    def __init__(self, dt: float):
        super().__init__(dt)

    def integrate(self, acc:BodyFixed, alpha:BodyFixed,q_rate:Quaternion, state:StateVector) -> StateVector:
        # Placeholder implementation: Simple Euler integration (this should be replaced with a more accurate method like RK4)
        new_state = StateVector(
            position=state.position + state.velocity * self.dt,
            velocity=state.velocity + acc * self.dt,
            quaternion=(state.quaternion + q_rate*self.dt).normalized,  # Orientation update would require quaternion integration
            omega=state.omega + alpha * self.dt
        )
        return new_state
        
    
# ── Environment ─────────────────────────────────────────────────────────────
class WindEffect(EnvironmentEffect):
    def __init__(self, dir: BodyFixed = BodyFixed(1.0, 0.0, 0.0), magnitude: float = 0.0):
        if np.linalg.norm(dir.vec) != 1:
            raise ValueError("Wind direction vector must be a unit vector.")
        
        self.force = dir * magnitude  # Wind force vector in the body-fixed frame

    def apply(self, state: StateVector) -> Tuple[BodyFixed, BodyFixed]:
        # Placeholder implementation: Return zero wind forces and moments
        return self.force, BodyFixed(0.0, 0.0, 0.0)

class DefaultEnvironment(EnvironmentBase):
    def __init__(self, effects: list[EnvironmentEffect] = [WindEffect()]):
        super().__init__(effects)


# ── Constraints ──────────────────────────────────────────────────────────────

class GroundPlaneConstraint(SetpointConstraint):
    """Prevents the drone from falling below a minimum altitude (z >= floor)."""

    def __init__(self, floor: float = 0.0):
        self.floor = floor

    def enforce(self, setpoint: Setpoints) -> StateVector:
        if setpoint.z < self.floor:
            setpoint.z = self.floor
        return setpoint


class MaxVelocityConstraint(StateConstraint):
    """Clamps the linear velocity magnitude to a maximum value."""

    def __init__(self, max_speed: float = 30.0):
        self.max_speed = max_speed

    def enforce(self, state: StateVector) -> StateVector:
        speed = float(np.linalg.norm(state.velocity.vec))
        if speed > self.max_speed:
            state.velocity.vec[:] = state.velocity.vec * (self.max_speed / speed)
        return state


class DefaultConstraints(ConstraintBase):
    """Ships with the standard set of constraints."""
    def __init__(self, constraints: list[StateConstraint | SetpointConstraint] | None = None):
        if constraints is None:
            constraints = [
                GroundPlaneConstraint(),
                MaxVelocityConstraint(),
            ]
        super().__init__(constraints)


# ── Drone ────────────────────────────────────────────────────────────────────


class DefaultDrone(DroneBase):
    def __init__(self, drone_id, init_state, pilot, dynamics, allocator, controller, integrator, environment, constraints=None):
        super().__init__(drone_id, init_state, pilot, dynamics, allocator, controller, integrator, environment, constraints)

    def get_setpoints(self):
        for iD in self.controller.channels:
            self.channels[iD] = self.controller.get_axis_value(iD)
        for iD in self.controller.switches:
            self.switches[iD] = self.controller.get_switch_value(iD)

        # Placeholder implementation: Return default setpoints based on controller input
        return Setpoints(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0
        )
    
    

    
