from quad_sim.bases.allocator import AllocatorBase, BodyFixed, Tuple
from quad_sim.bases.controller import ControllerBase, List
from quad_sim.bases.dynamics import DynamicsBase, RigidBody, MotorBase
from quad_sim.bases.pilot import PilotBase
from quad_sim.bases.drone import DroneBase, StateVector, Setpoints
from quad_sim.bases.integrator import IntegratorBase
from quad_sim.bases.environment import EnvironmentBase, EnvironmentEffect


class DefaultAllocator(AllocatorBase):
    def allocate(self, thrust_torques: Tuple[BodyFixed, BodyFixed]) -> list[float]:
        # Placeholder implementation: This should be replaced with actual allocation logic
        desired_thrust, desired_torque = thrust_torques
        # For demonstration, we return a list of zeros. Replace with actual calculations.
        return [0.0, 0.0, 0.0, 0.0]
    
class DefaultController(ControllerBase):
    def __init__(self):
        self.__connected = False

    def connect(self) -> bool:
        # Placeholder implementation: Simulate a successful connection
        return True
    
    def calibrate(self, min:int|dict, max:int|dict, trim:int|dict, offset:int|dict) -> bool:
        # Placeholder implementation: Simulate successful calibration
        return True
    
    def get_axis_value(self, channel_id:str|List[str]) -> float|dict:
        # Placeholder implementation: Return a default axis value
        return 0.0
    
    def get_switch_value(self, switch_id:str|List[str]) -> float|dict:
        # Placeholder implementation: Return a default switch value
        return 0.0
    
    def is_connected(self) -> bool:
        # Placeholder implementation: Simulate that the controller is always connected
        return self.__connected is True
    
class DefaultMotor(MotorBase):
    def __init__(self, id:str, spin_direction: int, position: BodyFixed):
        super().__init__(id, spin_direction, position)

        self.throttle = 0.0  # Initialize throttle to zero
        self.theta = 0.0     # Initialize propeller angle to zero

    def iD(self) -> str:
        return self._iD
    
    def compute_forces(self) -> Tuple[BodyFixed, BodyFixed]:
        # Placeholder implementation: Return zero thrust and moments
        return BodyFixed(0.0, 0.0, 0.0), BodyFixed(0.0, 0.0, 0.0)
    
    def set_rpm(self, rpm: float) -> None:
        # Placeholder implementation: Set the throttle based on RPM (this is just a dummy relationship)
        self.throttle = rpm / 10000.0  # Assume max RPM corresponds to full throttle


    def _generate_propeller_tips(self) -> dict[str, BodyFixed]:
        # Placeholder implementation: Return default propeller tip positions
        return {f"tip_{i}": BodyFixed(0.0, 0.0, 0.0) for i in range(4)}
    
    def locate_propeller_tips(self):
        return self._generate_propeller_tips()

    
class DefaultDynamics(DynamicsBase):
    def __init__(self, body: RigidBody, motors: List[MotorBase]):
        super().__init__(body, motors)
    
    @property
    def rotor_rates(self) -> dict[str, float]:
        # Placeholder implementation: Return default rotor rates
        return {f"motor_{i}": 0.0 for i in range(len(self.motors))}
    
class DefaultPilot(PilotBase):
    def compute_control(self, state: StateVector, target: Setpoints) -> Tuple[BodyFixed, BodyFixed]:
        # Placeholder implementation: Return zero thrust and moments
        return BodyFixed(0.0, 0.0, 0.0), BodyFixed(0.0, 0.0, 0.0)
    
class DefaultIntegrator(IntegratorBase):
    def __init__(self, dt: float):
        super().__init__(dt)

    def integrate(self, F:BodyFixed, M:BodyFixed, state:StateVector) -> StateVector:
        # Placeholder implementation: Return the input state without modification
        return state
    
class WindEffect(EnvironmentEffect):
    def apply(self, state: StateVector) -> Tuple[BodyFixed, BodyFixed]:
        # Placeholder implementation: Return zero wind forces and moments
        return BodyFixed(0.0, 0.0, 0.0), BodyFixed(0.0, 0.0, 0.0)

class DefaultEnvironment(EnvironmentBase):
    def __init__(self, effects: list[EnvironmentEffect] = [WindEffect()]):
        super().__init__(effects)


class DefaultDrone(DroneBase):
    def __init__(self, drone_id, init_state, pilot, dynamics, allocator, controller, integrator, environment):
        super().__init__(drone_id, init_state, pilot, dynamics, allocator, controller, integrator, environment)

    def get_setpoints(self):
        return Setpoints()
    
    

    
