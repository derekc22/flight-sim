#pragma once
#include <Eigen/Dense>
#include <functional>
#include "simulation/control/shared.hpp"

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
        const AxialControllerInput& axial_controller_input;
        const VelocityControllerInput& velocity_controller_input;
        const LinearFullStateFeedbackControllerInput& linear_full_state_feedback_controller_input;
        const NonlinearControllerInput& nonlinear_controller_input;
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

        ControlOutput step(const ControllerInput& controller_input, bool trim_bool);
    };

}

#include "simulation/control/pid/public.hpp"
#include "simulation/control/pid/controllers/axial/public.hpp"
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqt/public.hpp"
