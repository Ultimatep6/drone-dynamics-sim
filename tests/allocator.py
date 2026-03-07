import numpy as np
import pytest

from exampleSetup.default.config import (
    IntegratorConfig,
    LayoutConfig,
    MotorConfig,
    PhysicalLimits,
    RigidConfig,
    VehicleConfig,
)
from quad_sim.control.config.allocatorInput import AllocatorInput
from quad_sim.control.config.autopilotConf import AutopilotConfig
from quad_sim.control.config.flightMode import FlightMode
from quad_sim.dynamics.motors import PropMotor
from exampleSetup.default.state import StateVector
from quad_sim.simulation.drone import Drone

DEFAULT = VehicleConfig(
    vehicleID="testVehicle",
    physical_limits=PhysicalLimits(),
    layout_config=LayoutConfig(),
    rigid_config=RigidConfig(),
    pid_layout=AutopilotConfig(),
    integrator=IntegratorConfig(),
    init_state=StateVector(),
)


@pytest.fixture
def x_quad_setup():
    """Sets up a standard X-configuration quadcopter for testing."""
    kf = 3.13e-5
    km = 7.5e-7
    omega_min, omega_max = 1.0, 600.0**2

    # Motor Configs for the 4-rotor modular runner
    motors = [
        # Motor 0: Front-Right (+x, +y) | CW (-1)
        MotorConfig(k_f=kf, k_m=km, ang_range=(omega_min, omega_max)),
        # Motor 1: Front-Left (+x, -y) | CCW (+1)
        MotorConfig(k_f=kf, k_m=km, ang_range=(omega_min, omega_max)),
        # Motor 2: Rear-Right (-x, +y) | CCW (+1)
        MotorConfig(k_f=kf, k_m=km, ang_range=(omega_min, omega_max)),
        # Motor 3: Rear-Left (-x, -y) | CW (-1)
        MotorConfig(k_f=kf, k_m=km, ang_range=(omega_min, omega_max)),
    ]

    config = DEFAULT
    config.layout_config.rotors = motors
    drone = Drone(config)
    return drone


def test_pure_hover_balance(x_quad_setup):
    """Verify that pure thrust command distributes equally across all motors."""
    drone: Drone = x_quad_setup
    input_data = AllocatorInput(thrust=1.0, momentX=0.0, momentY=0.0, momentZ=0.0)
    w_sq = drone.allocator.calcRates(input_data)

    # All motors should have identical omega squared values

    assert np.allclose(w_sq, w_sq[0]), (
        "Hover thrust must be equal across all symmetrical motors."
    )


def test_pitch_ned_convention(x_quad_setup):
    drone = x_quad_setup
    input_data = AllocatorInput(thrust=1.0, momentX=0.0, momentY=0.1, momentZ=0.0)
    w_sq = drone.allocator.calcRates(input_data)

    # Get the x-positions of all motors
    x_positions = [m.position.vec[0, 0] for m in drone.getMotors()]

    # Logic: For Nose-Up (+My), speed must INCREASE as X DECREASES
    # Let's pick a motor in the Front (+x) and one in the Back (-x)
    front_idx = np.argmax(x_positions)
    back_idx = np.argmin(x_positions)

    assert w_sq[back_idx] > w_sq[front_idx], (
        f"Back motor (x={x_positions[back_idx]}) should be faster than "
        f"Front motor (x={x_positions[front_idx]}) for Nose-Up pitch."
    )


def test_stabilized_mode_shifting(x_quad_setup):
    """Verify that Stabilized mode shifts all motors up to prevent negative values."""
    drone = x_quad_setup
    drone.setMode(FlightMode.STABILIZED)

    # Low thrust with high roll would normally result in negative w^2
    input_data = AllocatorInput(thrust=0.05, momentX=1.0, momentY=0.0, momentZ=0.0)
    w_sq = drone.allocator.calcRates(input_data)

    assert np.min(w_sq) >= 0.0, (
        "Stabilized mode must shift values to stay above min_limit."
    )
    # Check that differences are preserved (Roll authority maintained)
    assert not np.allclose(w_sq, w_sq[0]), (
        "Roll authority should be preserved via shifting."
    )


def test_altitude_mode_scaling(x_quad_setup):
    """Verify that Altitude mode scales moments down to prioritize Thrust constraint."""
    drone = x_quad_setup

    # 1. Set to ALTITUDE mode to trigger the scaling logic
    drone.setMode(FlightMode.ALTITUDE)

    # 2. Get the actual physical limits from the allocator
    # (Assuming self.max_limits was set to 600^2 = 360000)
    max_limit = np.max(drone.allocator.max_limits)
    min_limit = np.min(drone.allocator.min_limits)

    # 3. Create a massive request that forces scaling
    # We use a mid-range thrust and huge moments
    input_data = AllocatorInput(thrust=5.0, momentX=50.0, momentY=50.0, momentZ=50.0)

    # 4. Calculate
    w_sq = drone.allocator.calcRates(input_data)

    # 5. Assertions
    # In Altitude mode, the scaling factor 's' MUST ensure no motor exceeds limits
    assert np.max(w_sq) <= max_limit + 1e-6, (
        f"Altitude mode failed! Motor at {np.max(w_sq)} exceeded limit {max_limit}"
    )
    assert np.min(w_sq) >= min_limit - 1e-6, (
        f"Altitude mode failed! Motor at {np.min(w_sq)} dropped below limit {min_limit}"
    )

    # 6. Verification of Priority
    # Ensure it didn't just 'zero out' the moments.
    # It should have scaled them to the maximum possible while keeping Thrust constant.
    assert not np.allclose(w_sq, w_sq[0]), (
        "Moments were completely wiped instead of scaled."
    )


def test_modular_n_rotor_support():
    """Verify that the allocator handles a Hexacopter (6 motors) using pseudoinverse."""
    kf, km = 1e-4, 1e-6
    omega_min, omega_max = 1.0, 600.0
    # 6 motors in a circle
    hex_motors = [
        MotorConfig(k_f=kf, k_m=km, ang_range=(omega_min, omega_max)) for i in range(6)
    ]
    config = DEFAULT
    config.layout_config.rotors = hex_motors
    drone = Drone(
        config,
    )
    try:
        alloc = drone.allocator
        input_data = AllocatorInput(thrust=1.0, momentX=0.1, momentY=0.0, momentZ=0.0)
        w_sq = alloc.calcRates(input_data)
        assert w_sq.shape == (6,), "Allocator failed to handle 6-motor configuration."
    except Exception as e:
        pytest.fail(f"Allocator failed N-rotor modularity check: {e}")


def test_math_round_trip(x_quad_setup):
    """Verifies that [MAT] * [w_sq] returns the original [Input]"""
    drone = x_quad_setup

    # Use a command that is well within limits to avoid saturation logic
    requested_wrench = np.array([10.0, 0.05, -0.05, 0.01])  # T, Mx, My, Mz
    input_data = AllocatorInput(*requested_wrench)

    w_sq = drone.allocator.calcRates(input_data)

    # Project the motor speeds back into the Wrench space
    # Force = MAT * w_sq
    actual_wrench = drone.allocator.MAT @ w_sq

    assert np.allclose(requested_wrench, actual_wrench, atol=1e-8), (
        f"Math inconsistency! Requested {requested_wrench} but got {actual_wrench}"
    )


def test_altitude_thrust_invariance(x_quad_setup):
    """In Altitude mode, scaling moments must NOT change total thrust."""
    drone = x_quad_setup
    drone.setMode(FlightMode.ALTITUDE)

    target_thrust = 5.0
    # Ask for a massive roll that triggers the scaling we just built
    input_data = AllocatorInput(
        thrust=target_thrust, momentX=100.0, momentY=0.0, momentZ=0.0
    )

    w_sq = drone.allocator.calcRates(input_data)

    # Calculate actual thrust produced: Sum(kf * w_sq)
    actual_thrust = np.sum(drone.allocator.MAT[0, :] * w_sq)

    assert np.isclose(actual_thrust, target_thrust), (
        f"Thrust was lost during scaling! Expected {target_thrust}, got {actual_thrust}"
    )


def test_boundary_zero_headroom(x_quad_setup):
    drone = x_quad_setup
    drone.setMode(FlightMode.ALTITUDE)

    # Set thrust to the absolute maximum possible (Sum of kf * max_w2)
    max_w2 = 600.0**2
    max_thrust = len(drone.getMotors()) * (3.13e-5 * max_w2)

    # Request max thrust + some roll
    input_data = AllocatorInput(
        thrust=max_thrust, momentX=10.0, momentY=0.0, momentZ=0.0
    )
    w_sq = drone.allocator.calcRates(input_data)

    print(w_sq)

    # Every motor should be pegged at max_w2, and moments should be 0
    assert np.allclose(w_sq, max_w2), "At max thrust, moments must scale to zero."


def test_visual_inspection(x_quad_setup):
    """Prints a table of motor outputs for different scenarios."""
    drone = x_quad_setup
    scenarios = {
        "Hover": AllocatorInput(2.0, 0, 0, 0),
        "Full Roll": AllocatorInput(2.0, 0.5, 0, 0),
        "Full Pitch": AllocatorInput(2.0, 0, 0.5, 0),
        "Full Yaw": AllocatorInput(2.0, 0, 0, 0.05),
    }

    print(
        f"\n{'Scenario':<15} | {'M0 (FR)':<10} | {'M1 (FL)':<10} | {'M2 (RR)':<10} | {'M3 (RL)':<10}"
    )
    print("-" * 65)

    for name, cmd in scenarios.items():
        w_sq = drone.allocator.calcRates(cmd)
        # Format for display: take sqrt to show rad/s
        w = np.sqrt(np.maximum(w_sq, 0))
        print(
            f"{name:<15} | {w[0]:<10.1f} | {w[1]:<10.1f} | {w[2]:<10.1f} | {w[3]:<10.1f}"
        )
