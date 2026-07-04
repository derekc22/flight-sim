#pragma once
#include <functional>
#include "simulation/control/shared/public.hpp"

namespace control {

    enum class ControllerType {
        None,
        // Attitude
        AttitudePID,
        DamperPID,
        // Velocity
        VelocityPID,
        // Linear Quadratic
        LinearQuadraticRegulator,
        LinearQuadraticTracker,
        LinearQuadraticIntegrator,
        // Nonlinear
        FeedbackLinearization,
        NonlinearDynamicInversion,
        IncrementalNonlinearDynamicInversion
    };

    using AttitudeController = std::function<ControlOutput(const AttitudeControllerInput&, double dt)>;

    using VelocityController = std::function<ControlOutput(const VelocityControllerInput&, double dt)>;

    using LinearQuadraticController = std::function<ControlOutput(const LinearQuadraticControllerInput&, double dt)>;

    using NonlinearController = std::function<ControlOutput(const NonlinearControllerInput&, double dt)>;

    struct ControllerInputs {
        const AttitudeControllerInput& attitude_controller_input;
        const VelocityControllerInput& velocity_controller_input;
        const LinearQuadraticControllerInput& linear_quadratic_controller_input;
        const NonlinearControllerInput& nonlinear_controller_input;
    };

    struct ControlProperties {
        ControllerType attitude_controller_type = ControllerType::None;
        ControllerType velocity_controller_type = ControllerType::None;
        ControllerType linear_quadratic_controller_type = ControllerType::None;
        ControllerType nonlinear_controller_type = ControllerType::None;

        AttitudeController attitude_controller;
        VelocityController velocity_controller;
        LinearQuadraticController linear_quadratic_controller;
        NonlinearController nonlinear_controller;

        ControlOutput step(const ControllerInputs& inputs, double dt, bool trim_flag);
    };

}
