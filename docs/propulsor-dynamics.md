# Propulsor Dynamics

## Current Model

The current model treats thrust as the propulsor actuator output.

1. `ControlManager` produces a desired aircraft body force and body moment.
2. `AllocatorManager` converts that desired wrench into a commanded thrust for each propulsor.
3. `ActuatorManager` applies limits and a first-order lag to each commanded thrust.
4. `PropulsionManager` receives the resulting actual thrust for each propulsor.
5. Each propulsor calculates `omega` directly from its actual thrust.
6. Each propulsor estimates `omega_dot` from the current and previous calculated `omega` values.
7. `PropulsionManager` calculates thrust, propeller torque, rotor-acceleration reaction moment, and gyroscopic moment.
8. The rigid-body integrator receives the total propulsive body force and body moment.

The current signal flow is:

```text
desired body force [N] and moment [N*m]
    -> commanded thrust per propulsor [N]
    -> lagged actual thrust per propulsor [N]
    -> calculated omega per propulsor [rad/s]
    -> propulsive body force [N] and moment [N*m]
```

`omega` is not an independently integrated state. The model prescribes thrust and calculates the rotor speed required to produce that thrust.

## Second-Order Model Clarification

### Why "Second Order" Was Ambiguous

The canonical second-order transfer function is not ambiguous by itself. The ambiguity is which physical quantity is its input and which physical quantity is its output.

The current first-order model has an explicit mapping:

```text
commanded thrust -> first-order actuator -> actual thrust
```

A second-order model could instead describe commanded thrust to actual thrust, desired rotor speed to actual rotor speed, or requested shaft torque to actual shaft torque. Those choices produce different states, module boundaries, and physical assumptions. They must not be combined as if they were the same model.

### Possible Second-Order Models

#### Model 1: Second-Order Thrust Actuator

```text
commanded thrust -> second-order actuator -> actual thrust
actual thrust -> calculated omega
```

The two states for each propulsor are:

- actual thrust
- thrust rate

The canonical state equations are:

```text
actual_thrust_dot = thrust_rate

thrust_rate_dot =
    natural_frequency^2 * (commanded_thrust - actual_thrust)
    - 2 * damping_ratio * natural_frequency * thrust_rate
```

The transfer-function input and output are both thrust, so the model has unity steady-state gain and no unit conversion inside the transfer function.

`omega` remains an algebraic value calculated from actual thrust. The actuator's internal thrust-rate state does not cross the actuator-manager boundary, so propulsion continues to calculate `omega_dot` by differencing the current and previous `omega` values.

This model:

- directly replaces the existing first-order thrust actuator
- preserves commanded thrust as the allocator interface
- remains powerplant-agnostic
- does not require a speed controller, drive, motor, engine, or shaft-torque response model
- does not make `omega` an independently integrated state

#### Model 2: Second-Order Rotor-Speed Response

```text
commanded thrust
    -> desired omega
    -> second-order speed response
    -> actual omega
    -> actual thrust
```

The two states for each propulsor are:

- `omega`
- `omega_dot`

This model treats the speed controller and powerplant as a combined black box. It assumes the propulsor behaves like a speed-governed system. It does not calculate rotor speed from a physical shaft-torque balance; required shaft torque is inferred afterward and may be unlimited.

This assumption is not universal for fixed-pitch piston installations, governed variable-pitch propellers, turboprops, and turbofan spools.

#### Model 3: Torque-Driven Rotor

```text
powerplant command
    -> shaft torque
    -> rotor torque balance
    -> omega
    -> thrust
```

The rotor equation is first order in `omega`:

```text
omega_dot =
    (shaft_torque - aerodynamic_resisting_torque)
    / rotor_inertia
```

Making the complete system second order requires another independent state. One possible model is:

```text
requested shaft torque
    -> first-order torque response
    -> actual shaft torque
    -> rotor torque balance
    -> omega
```

The two states would then be actual shaft torque and `omega`. This model requires decisions about the abstract or concrete torque-producing subsystem.

### Incompatible Combination to Avoid

A complete second-order transfer function from desired `omega` to actual `omega` must not be combined with additional drive dynamics and separate torque-balance integration of `omega`. That would count the rotor response more than once and make the complete system higher than second order.

Likewise, placing a second-order drive in front of an independently integrated rotor-speed state produces a system with at least three states, not a second-order command-to-speed model.

### Recommended Model

Use model 1: the second-order thrust actuator.

This recommendation follows from the current architecture and the intended scope:

- The allocator already commands thrust.
- The current actuator already maps commanded thrust to actual thrust.
- The model should remain independent of electric, piston, turbine, or other powerplant details.
- The initial implementation supports only the existing constant-coefficient propeller model.

The proposed signal flow is:

```text
desired body force [N] and moment [N*m]
    -> commanded thrust per propulsor [N]
    -> second-order thrust actuator with internal thrust and thrust-rate states
    -> actual thrust [N]
    -> calculated omega [rad/s]
    -> finite-difference omega_dot [rad/s^2]
    -> propulsive body force [N] and moment [N*m]
```

The resulting state changes are two internal actuator states per propulsor: actual thrust and thrust rate. `omega` and `omega_dot` are derived propulsion quantities, not additional states. Surface actuators use the same generic implementation with actual deflection and deflection-rate states.

### Initialization

Initialize each propulsor's actual thrust to its initial commanded or trimmed thrust and initialize thrust rate to zero.

For a zero-thrust initialization:

```text
actual thrust = 0 N
thrust rate = 0 N/s
```

For a trimmed initialization:

```text
actual thrust = trimmed thrust
thrust rate = 0 N/s
```

This prevents an artificial actuator transient at initialization. The corresponding initial `omega` is calculated from the initialized actual thrust.

### Parameters and Limits

Each propulsor needs these second-order parameters:

- natural frequency in rad/s
- dimensionless damping ratio

They should be configurable per propulsor rather than hardcoded.

The canonical transfer function is linear and unconstrained, while the existing actuators have minimum and maximum thrust limits. Clamping commanded thrust does not guarantee that actual thrust remains within those limits because an underdamped response may overshoot.

A saturation policy must therefore be selected during implementation. Projecting actual thrust or thrust rate onto a limit changes the exact canonical response, while leaving the state unconstrained permits temporary limit violations. This decision is separate from the choice of natural frequency and damping ratio.

### Powerplant Boundary

The recommended model does not define a powerplant interface. It does not require voltage, current, throttle, fuel flow, engine torque, turbine torque, or actual shaft torque as an actuator input or output.

Shaft torque may still be inferred for load accounting:

```text
inferred shaft torque =
    rotor inertia * omega_dot
    + aerodynamic resisting torque
```

Inferring shaft torque does not create a powerplant model and does not constrain the aircraft to an electric, piston, turbine, or other powerplant.

## Implementation Scope

The second-order value and rate states remain internal to each actuator component, matching the ownership of the current first-order lag state. Runtime advances those internal states, and trim initializes the actual value to the trimmed command and the rate to zero.

The reduced aircraft model continues to treat actual actuator outputs as inputs. Its state dimensions, automatic differentiation, linearization, estimation, control, allocator, rigid-body integrator, and propulsion interfaces are not augmented with actuator states.

The initial implementation is limited to the existing constant-coefficient propeller model. Variable-pitch propellers, turbofan maps, and powerplant-specific dynamics remain outside this scope.

No build or test command is authorized for this change.

## Questions and Answers

### 1. Does this generalize to the major types of powered conventional aircraft, including turbofans, turboprops, and piston-engine aircraft?

The recommended second-order thrust actuator generalizes at the control-oriented thrust-response level because it does not depend on how a powerplant produces thrust.

The current constant-coefficient propeller calculation does not generalize to every rotor type. In particular:

- A fixed-pitch piston-engine propeller may use propeller coefficient data that varies with advance ratio.
- A constant-speed piston or turboprop installation normally includes a governor and variable blade pitch, so rotor speed alone does not uniquely determine thrust.
- A turbofan may contain multiple coupled shafts or spools and requires fan or engine performance maps rather than the current propeller equation.

The thrust-actuator interface can remain common, but each future propulsion type needs its own model for any internal quantities that the simulation is expected to reproduce. The initial implementation supports only the existing constant-coefficient propeller model.

If a physical torque-driven model is added later, its high-level torque balance generalizes:

```text
driving shaft torque
    - aerodynamic load torque
    = torque available to accelerate the rotating assembly
```

This balance applies regardless of whether the power ultimately comes from an electric motor, piston engine, or gas turbine, but it is not part of the recommended initial refactor.

### 2. Does speed controller literally mean a controller such as a PID controller?

It could mean a PI controller, PID controller, state-feedback controller, gain-scheduled controller, or abstract governor.

The recommended second-order thrust actuator does not include a speed controller. A speed controller belongs to model 2 and would add the assumption that the propulsor is governed to follow desired `omega`.

### 3. What is a drive, and is this where the second-order dynamics come into play?

A drive is an abstract torque-producing subsystem. It would receive a request and produce actual shaft torque subject to its response dynamics and limits.

The recommended model does not include a drive. Its second-order dynamics are entirely the commanded-thrust-to-actual-thrust actuator response.

In a future torque-driven model, a first-order drive plus the first-order rotor-speed state would make the complete request-to-`omega` system second order. A second-order drive plus an independently integrated rotor-speed state would make it at least third order.

### 4. Is a propeller or fan model required to calculate desired `omega` from commanded thrust?

The recommended model does not calculate desired `omega`. Its actuator evolves actual thrust directly.

A propeller model is still required to calculate actual `omega`. The existing finite difference of current and previous `omega` remains responsible for `omega_dot`, which is used for the rotor-acceleration reaction moment.

The current model uses a constant-coefficient propeller relationship. It can calculate actual `omega` from actual thrust, but it is not a universal propeller or fan model.

- A fixed-pitch propeller generally needs thrust and torque coefficient data as functions of advance ratio and possibly Mach and Reynolds number.
- A variable-pitch propeller also needs blade pitch. A requested thrust does not uniquely determine `omega` unless a governor or pitch-selection rule is defined.
- A turbofan generally needs fan or engine performance maps. The current propeller relationship is not sufficient.

The initial implementation remains limited to the existing constant-coefficient model.

### 5. What are shaft torque and aerodynamic resisting torque, and is either one also called drag torque?

Shaft torque is the driving torque transmitted from the drive or powerplant to the rotating propeller or fan.

Aerodynamic resisting torque is the opposing torque exerted by the air on the rotating propeller or fan as the rotor transfers energy to the airflow. It is also commonly called aerodynamic torque, load torque, propeller torque, or drag torque.

Shaft torque is not drag torque. They act in opposite rotational directions. At constant rotor speed, their magnitudes balance when other losses are neglected. When shaft torque is greater, `omega` increases; when aerodynamic resisting torque is greater, `omega` decreases.

Under the recommended thrust-actuator model, shaft torque is inferred from the prescribed thrust response rather than used to drive that response. Aerodynamic resisting torque remains the drag or load torque calculated by the propeller model.
