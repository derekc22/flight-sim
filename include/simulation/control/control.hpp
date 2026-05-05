#pragma once
#include <Eigen/Dense>
#include <functional>
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"
#include "simulation/control/pid/controllers/damper.hpp"
#include "simulation/control/pid/controllers/velocity.hpp"
#include "simulation/control/linear_quadratic/linear_quadratic.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt.hpp"

namespace control {

    enum class ControlType {
        None,
        // Axial
        AxialPID,
        DamperPID,
        // Velocity
        VelocityPID,
        // Linear Full State Feedback
        LinearQuadraticRegulator,
        LinearQuadraticTracker,
        LinearQuadraticIntegrator,
        // Nonlinear
        FeedbackLinearization,
        NonlinearDynamicInversion,
        IncrementalNonlinearDynamicInversion
    };

    using AxialController = std::function<ControlOutput(const AxialControllerInput&)>;

    using VelocityController = std::function<ControlOutput(const VelocityControllerInput&)>;

    using LinearFullStateFeedbackController = std::function<ControlOutput(const LinearFullStateFeedbackControllerInput&)>;

    using NonlinearController = std::function<ControlOutput(const NonlinearControllerInput&)>;

    struct ControllerInput {
        AxialControllerInput axial_controller_input;
        VelocityControllerInput velocity_controller_input;
        LinearFullStateFeedbackControllerInput linear_full_state_feedback_controller_input;
        NonlinearControllerInput nonlinear_controller_input;
    };

    struct ControlProperties {
        ControlType axial_control_type = ControlType::None;
        ControlType velocity_control_type = ControlType::None;
        ControlType linear_full_state_feedback_control_type = ControlType::None;
        ControlType nonlinear_control_type = ControlType::None;

        AxialController axial_controller;
        VelocityController velocity_controller;
        LinearFullStateFeedbackController linear_full_state_feedback_controller;
        NonlinearController nonlinear_controller;

        ControlOutput step(const ControllerInput& ctrl_law_input, bool trim_bool);
    };

    Eigen::VectorXd unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs& u_surface);
    Eigen::VectorXd unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs& u_propulsor);
    Eigen::VectorXd unpack_actuator_inputs(const SurfaceActuatorInputs& u_surface, const PropulsorActuatorInputs& u_propulsor);
}
