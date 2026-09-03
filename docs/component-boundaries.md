# Manager and Component Boundaries

Components represent distinct functional responsibilities with their own behavior and, when applicable, state. The `Component` suffix is omitted because the `components/` directory already communicates that role.

| Manager | Components | Boundary |
|---|---|---|
| `ActuatorManager` | `ElevatorActuator`<br>`AileronActuator`<br>`RudderActuator`<br>`FlapActuator`<br>`SpoilerActuator`<br>`FrontPropulsorActuator`<br>`LeftPropulsorActuator`<br>`RightPropulsorActuator` | Each actuator is independently configured and owns its own command limiting and lag state. |
| `EstimationManager` | `LinearKalmanEstimator`<br>`ExtendedKalmanEstimator` | The estimators have distinct inputs, algorithms, and persistent filter state. |
| `ControlManager` | `AttitudeControl`<br>`VelocityControl`<br>`FullStateControl` | These are the major control responsibilities. Specific PID and linear-quadratic controllers are private implementations owned by the applicable component. |
| `SensorManager` | `AngleOfAttackVane`<br>`Accelerometer`<br>`Gyroscope`<br>`PitotTube`<br>`StaticPort`<br>`TotalAirTemperatureProbe`<br>`GNSSReceiver`<br>`Magnetometer` | Each sensor independently owns its measurement behavior, noise, bias, and lag state. |
| `AvionicsManager` | `AirDataComputer`<br>`AttitudeHeadingReferenceSystem`<br>`InertialNavigationSystem` | These are distinct avionics processors with separate responsibilities and state. |
| `GuidanceManager` | `RegulationGuidance`<br>`TrackingGuidance`<br>`InterpolatedGuidance` | Regulation, tracking, and interpolation are semantically distinct guidance modes and therefore distinct components. |
| `AerodynamicsManager` | None | The manager owns surface definitions and orchestrates stateless aerodynamic state and load calculations in `detail/`. |
| `AllocatorManager` | None | Effectiveness evaluation and constrained or unconstrained solving are implementation stages of one cohesive allocation responsibility, so they remain private detail calculations. |
| `StructuralManager` | None | Deriving mass properties from structural geometry is one cohesive responsibility. Components could become appropriate if independently behaving structural subsystems are added later, such as fuel mass, releasable payload, moving landing gear, structural damage, or other time-varying mass sources. |
| `PropulsionManager` | None | The manager owns persistent propulsion state and orchestrates the closely coupled propeller-state and propulsive-load calculations in `detail/`. |

## Runner Wrappers

`Runner` is the top-level simulation orchestrator, not a subsystem manager. Its `TrimWrapper`, `LinearizationWrapper`, `MeasurementsWrapper`, `EstimationWrapper`, `ControlWrapper`, `PhysicsWrapper`, and `RecordingWrapper` coordinate the subsystem managers and exchange per-step data through explicit payloads and `StepContext`. `Scheduler` owns runner timing, while `FlightGearAdapter` handles the external FlightGear integration. `Runner` owns and steps `FSMManager` directly, then supplies the selected mode to the wrappers that consume it.

## Granularity Rules

- Components are not individual formulas, repeated data entries, or intermediate stages of one calculation.
- Individual geometries are data, not structural components.
- Aerodynamic kinematics, coefficients, and loads remain calculation stages rather than components.
- PID and linear-quadratic algorithms remain implementations inside their owning control component.
- Constrained and unconstrained allocation remain solver paths rather than separate components.
