# Manager and Component Boundaries

Components represent distinct functional responsibilities with their own behavior and, when applicable, state. The `Component` suffix is omitted because the `components/` directory already communicates that role.

| Manager | Components | Boundary |
|---|---|---|
| `ActuatorManager` | `ElevatorActuator`<br>`AileronActuator`<br>`RudderActuator`<br>`FlapActuator`<br>`SpoilerActuator`<br>`FrontPropulsorActuator`<br>`LeftPropulsorActuator`<br>`RightPropulsorActuator` | Each actuator is independently configured and owns its own command limiting and lag state. |
| `EstimationManager` | `LinearKalmanFilter`<br>`ExtendedKalmanFilter` | The filters have distinct inputs, algorithms, and persistent state. |
| `ControlManager` | `AttitudeControl`<br>`VelocityControl`<br>`LinearQuadraticControl`<br>`NonlinearControl` (not implemented) | These are the major control responsibilities. Specific controller implementations are grouped under Control's private `components/controllers/` category, while their reusable policy components are grouped under `components/policies/`. |
| `SensorManager` | `AngleOfAttackVane`<br>`Accelerometer`<br>`Gyroscope`<br>`PitotTube`<br>`StaticPort`<br>`TotalAirTemperatureProbe`<br>`GNSSReceiver`<br>`Magnetometer` | Each sensor independently owns its measurement behavior, noise, bias, and lag state. |
| `AvionicsManager` | `AirDataComputer`<br>`AttitudeHeadingReferenceSystem`<br>`InertialNavigationSystem` | These are distinct avionics processors with separate responsibilities and state. |
| `GuidanceManager` | `RegulationGuidance`<br>`TrackingGuidance`<br>`InterpolatedGuidance` | Regulation, tracking, and interpolation are semantically distinct guidance modes and therefore distinct components. |
| `AerodynamicsManager` | None | The manager owns surface definitions and orchestrates stateless aerodynamic load calculations in `detail/`. Aerodynamic state remains a stateless detail calculation used independently by the runner. |
| `AllocatorManager` | None | Effectiveness evaluation and constrained or unconstrained solving are implementation stages of one cohesive allocation responsibility, so they remain private detail calculations. |
| `StructuralManager` | None | Deriving mass properties from structural geometry is one cohesive responsibility. Components could become appropriate if independently behaving structural subsystems are added later, such as fuel mass, releasable payload, moving landing gear, structural damage, or other time-varying mass sources. |
| `PropulsionManager` | None | The manager owns persistent propulsion state and orchestrates the closely coupled propeller-state and propulsive-load calculations in `detail/`. |
| `RunManager` | `TrimWrapper`<br>`LinearizationWrapper`<br>`MeasurementsWrapper`<br>`EstimationWrapper`<br>`ControlWrapper`<br>`PhysicsWrapper`<br>`RecordingWrapper`<br>`Scheduler`<br>`FlightGearAdapter` | The manager performs top-level simulation orchestration. Its components coordinate subsystem managers, exchange per-step data through explicit payloads and `StepContext`, schedule execution, and handle the external FlightGear connection. |

Within the RunManager component set, the wrappers are grouped under `public/components/wrappers/`. The `Wrapper` suffix identifies a component that wraps an existing module interface and adapts it to the run's step and data flow. The wrapped interface may be a manager or a free-function API. `Scheduler` and `FlightGearAdapter` remain directly under `public/components/` and do not use the suffix because they directly implement scheduling and the external FlightGear boundary rather than wrapping another module interface. `wrappers/` is a RunManager component category, not a repository-wide layout category.

## Granularity Rules

- Components are not individual formulas, repeated data entries, or intermediate stages of one calculation.
- Individual geometries are data, not structural components.
- Aerodynamic kinematics, coefficients, and loads remain calculation stages rather than components.
- PID and linear-quadratic algorithms remain implementations inside their owning control component.
- Constrained and unconstrained allocation remain solver paths rather than separate components.
