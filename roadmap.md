## Plan: Modular Architecture Implementation Roadmap

The framework's foundation is solid, but to achieve the 2026 research goals of multi-agent simulation and pluggable components, a more formal, decoupled architecture is required. This roadmap outlines the implementation of a new architecture based on Abstract Base Classes (ABCs) and a two-tiered configuration pattern.

---

### Phase 1 — Implement Core Architectural Patterns

This phase establishes the foundational contracts and patterns that will enable modularity and extensibility.

1.  **Define the Hierarchy of Abstractions (ABCs):** Create a new `quad_sim/templates/` directory to house all abstract base classes. Define the core contracts for the simulation:
    *   `SimBase`: The top-level simulation runner.
    *   `DroneBase`: The container for a single agent.
    *   `PilotBase`: For flight controllers (the "brain").
    *   `DynamicsBase`: For physics models (the "body").
    *   `IntegratorBase`: For numerical integration methods.
    *   `AllocatorBase`: For control signal mixing.
    *   `EnvironmentBase`: For external world conditions (gravity, wind).
    *   **`FlightModeStrategyBase`**: For encapsulating the logic of a specific flight mode (e.g., Acro, Level, Position). This strategy will be responsible for both calculating setpoints and declaring the motor desaturation priority for that mode.

2.  **Implement the Two-Tier Configuration Pattern:**
    *   **Create `BuildableConfig`:** An ABC for high-level configs that must have a `.build()` method. `VehicleConfig` will implement this to construct a complete `Drone` object.
    *   **Create `DataConfig`:** A simple Pydantic-based class for low-level, data-only configurations. All component configs (`AutopilotConfig`, `LayoutConfig`, `RigidConfig`, etc.) will inherit from this.
    *   **Refactor `config.py`:** Update all configuration classes to inherit from the appropriate new base (`BuildableConfig` or `DataConfig`).

3.  **Refactor `nCopterSimulator` into the `SimBase` Implementation:**
    *   Modify the simulator's constructor to be the "master builder". It will now accept a top-level configuration, iterate through the `VehicleConfig`s, and call `config.build()` on each to get fully assembled `DroneBase` objects.
    *   The simulator's main loop will be simplified to call `drone.step()` on each agent.

### Phase 2 — Refactor Subsystems to Adhere to Contracts

With the core patterns in place, refactor existing code to conform to the new, stricter interfaces.

4.  **Refactor `AutoPilot` and `Allocator`:**
    *   Make `AutoPilot` inherit from `PilotBase`. It will use the **Strategy Pattern** to manage a dictionary of `FlightModeStrategyBase` objects.
    *   Make `Allocator` inherit from `AllocatorBase`. Its `allocate` method will now accept the desaturation priority from the active flight mode strategy to handle motor limits correctly.

5.  **Refactor `Quadcopter` and `integrators.py`:**
    *   Make `Quadcopter` inherit from `DynamicsBase`, implementing the required `compute_forces_and_moments` method.
    *   Make `ForwardEulerIntegrator` inherit from `IntegratorBase`.
    *   Decouple the integrator from the drone model. The `step` method should accept `(state, derivatives, dt)` as pure inputs, rather than reaching into the drone object.

6.  **Implement `environment.py`:**
    *   Create a concrete `SimpleEnvironment` class that implements `EnvironmentBase`.
    *   The `SimBase` runner will query the environment for external forces (like gravity and wind) and pass them to the `DynamicsBase` component of each drone during the step loop. This replaces the hardcoded `9.81` gravity.

### Phase 3 — Advanced Features & Observability

Build upon the new modular architecture to add advanced capabilities.

7.  **Add an Inter-Drone Interaction Layer:** Inside the `SimBase` step loop, add an optional `InteractionModel` (e.g., for downwash or collision avoidance). After all drones compute their primary forces but before integration, this model can modify each drone's state based on its neighbors.

8.  **Enhance and Standardize the Logging System:** Refactor the logging mechanism to be more robust and extensible.
    *   **Create a Central `Logger` Class:** This class will be initialized by `SimBase` and will be responsible for collecting data from all simulation components.
    *   **Standardize the `@loggable` Decorator:** Ensure all refactored subsystems use the decorator consistently for data exposure.
    *   **Implement Logging Backends:** The central `Logger` should support different output strategies (e.g., writing to CSV, HDF5, or a live plotting buffer) to make data analysis more flexible.
    *   **Ensure Multi-Agent Support:** The logger must be able to handle multiple drones, for example by automatically prefixing logged data columns with the `drone.id`.

9.  **Wire up `viz/`:** Define a `Visualizer` interface. The `SimBase` runner will have a `run()` method that can accept a visualizer instance to provide real-time (matplotlib) or post-hoc (HDF5) plotting.

---

### Further Considerations

1.  **Integrator Registry:** With the `IntegratorBase` in place, adding new integrators like RK4 becomes a matter of creating a new class that implements the interface. This should be prioritized after Phase 2.
2.  **Config Co-location:** The monolithic `config.py` should be split. Each subsystem's `DataConfig` should be moved to a `config.py` file within its own module (e.g., `quad_sim/control/config.py`), mirroring the code's structure.
3.  **Position Mode:** The unimplemented `POSITION` flight mode in `AutoPilot` should be addressed by creating a `PositionModeStrategy` class as part of the work in item #4.
4.  **CAD model importing:** The unimplemented `POSITION` flight mode in `AutoPilot` should be addressed by creating a `PositionModeStrategy` class as part of the work in item #4.
