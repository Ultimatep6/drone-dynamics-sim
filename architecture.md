# Simulation Architecture Plan

This document outlines the modular, extensible architecture for the drone simulation framework. The design is centered around Abstract Base Classes (ABCs) and a two-tiered configuration pattern to ensure a clear separation of concerns.

## 1. Core Principles

-   **Modularity:** The simulation is broken down into distinct, interchangeable components (e.g., Pilot, Dynamics, Integrator).
-   **Extensibility:** Users can add new components (like custom controllers or physics models) by adhering to the defined abstract contracts.
-   **Decoupling:** High-level components (like the `Simulator`) are decoupled from the concrete implementations of the components they manage.

## 2. Hierarchy of Abstractions

The system is defined by a clear hierarchy of Abstract Base Classes (ABCs). Each ABC defines a "contract" for a specific role within the simulation.

-   `SimBase` (The "World")
    -   The top-level class that manages the entire simulation.
    -   **Contains:**
        -   An `EnvironmentBase` object (defining gravity, wind, etc.).
        -   A list of `DroneBase` objects (the agents in the simulation).
    -   **Responsibilities:** Manages the main simulation loop, stepping each drone forward in time.

-   `DroneBase` (The "Agent")
    -   Represents a single autonomous agent (e.g., a quadcopter, a fixed-wing plane).
    -   Acts as a container that connects the "brain" to the "body".
    -   **Contains:**
        -   `PilotBase`: The flight controller or "brain".
        -   `DynamicsBase`: The physics model or "body".
        -   `IntegratorBase`: The numerical integrator that advances the state through time.
        -   `AllocatorBase`: The mixer that translates pilot commands to actuator outputs.
    -   **Responsibilities:** Coordinates the execution of its internal components during a single time step.

## 3. The Two-Tier Configuration Pattern

To balance flexibility with simplicity, the architecture uses a two-tiered configuration system. This separates the responsibility of *holding data* from the responsibility of *constructing objects*.

### 3.1. Tier 1: The "Builder" (`BuildableConfig`)

-   **Role:** A high-level configuration responsible for constructing a complex, composite object like a `Drone`.
-   **Contract:** An ABC that requires a `.build()` method.
-   **Example:** `VehicleConfig` is a `BuildableConfig`. Its `.build()` method reads data from its internal, lower-tier configs to instantiate and assemble the `Pilot`, `Dynamics`, and `Integrator` objects into a complete `Drone`.

### 3.2. Tier 2: The "Data Container" (`DataConfig`)

-   **Role:** A low-level configuration that holds and validates parameters for a specific, granular component.
-   **Contract:** A simple base class (e.g., using Pydantic) with no build logic.
-   **Example:** `AutopilotConfig`, `LayoutConfig`, and `RigidConfig` are `DataConfig`s. They are simple data holders used by the `VehicleConfig` during the build process.

### 3.3. Example Flow

1.  The `Simulator` is initialized with a top-level `NCopterConfig`.
2.  It iterates through the list of `VehicleConfig` objects within `NCopterConfig`.
3.  For each `VehicleConfig`, the simulator calls `drone_config.build()`.
4.  The `build()` method inside `VehicleConfig` reads its internal `DataConfig`s (e.g., `self.pid_config`, `self.layout_config`).
5.  It uses this data to instantiate the concrete `AutoPilot`, `Quadcopter`, and `ForwardEulerIntegrator` objects.
6.  It assembles these objects into a new `Drone` instance and returns it to the simulator.

This approach centralizes complex construction logic in the `BuildableConfig` while keeping component-level configurations as simple, validated data containers.

## 4. Component ABCs

The following ABCs will form the primary contracts for the simulation framework.

| ABC              | Role                                                              | Key Abstract Methods/Properties                               |
| ---------------- | ----------------------------------------------------------------- | ------------------------------------------------------------- |
| `PilotBase`      | The "brain". Calculates desired control actions.                  | `compute_control(state, target)`                              |
| `DynamicsBase`   | The "body". Calculates forces and moments from actuator commands. | `compute_forces_and_moments(actuators)`, `mass`, `inertia`     |
| `IntegratorBase` | The "rules of time". Advances the drone's state.                  | `step(state, derivatives, dt)`                                |
| `AllocatorBase`  | The "nervous system". Maps pilot commands to actuator outputs.    | `allocate(commands, num_actuators)`                           |
| `SensorBase`     | Provides measurements of the state (e.g., IMU, GPS).              | `measure(true_state)`                                         |
| `EnvironmentBase`| Defines the physical world.                                       | `get_gravity()`, `get_wind(position)`                         |

This architecture ensures a robust, scalable, and user-friendly foundation for all future development.