#pragma once
#include <functional>
#include <optional>
#include "simulation/control/shared/public.hpp"
#include "simulation/trim/public.hpp"

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

    using AttitudeController = std::function<VirtualControlOutput_T<double>(const AttitudeControllerInput&, double dt)>;

    using VelocityController = std::function<VirtualControlOutput_T<double>(const VelocityControllerInput&, double dt)>;

    using LinearQuadraticController = std::function<VirtualControlOutput_T<double>(const LinearQuadraticControllerInput&, double dt)>;

    using NonlinearController = std::function<VirtualControlOutput_T<double>(const NonlinearControllerInput&, double dt)>;

    struct ControllerInputs {
        std::optional<AttitudeControllerInput> attitude_controller_input;
        std::optional<VelocityControllerInput> velocity_controller_input;
        std::optional<LinearQuadraticControllerInput> linear_quadratic_controller_input;
        std::optional<NonlinearControllerInput> nonlinear_controller_input;
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

        VirtualControlOutput step(const ControllerInputs& inputs, double dt);

        ControllerInputs build_controller_inputs(
            const dynamics::RigidBodyState& Zt, 
            const trim::TrimSolution& trim_sol, 
            const linearization::VirtualLocalLinearization& virtual_lin_sol, 
            const actuators::SurfaceActuators& surface_actuators, 
            const actuators::PropulsorActuators& propulsor_actuators, 
            const guidance::GuidanceSetpoint& setpoint
        );
    };

}
