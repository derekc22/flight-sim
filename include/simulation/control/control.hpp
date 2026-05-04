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

    using AxialControlLaw = std::function<ControlOutput(const AxialControlLawInput&)>;

    using VelocityControlLaw = std::function<ControlOutput(const VelocityControlLawInput&)>;

    using LinearFullStateFeedbackControlLaw = std::function<ControlOutput(const LinearFullStateFeedbackControlLawInput&)>;

    using NonlinearControlLaw = std::function<ControlOutput(const NonlinearControlLawInput&)>;

    struct ControlLawInput {
        AxialControlLawInput axial_control_law_input;
        VelocityControlLawInput velocity_control_law_input;
        LinearFullStateFeedbackControlLawInput linear_full_state_feedback_control_law_input;
        NonlinearControlLawInput nonlinear_control_law_input;
    };

    struct ControlProperties {
        ControlType axial_control_type = ControlType::None;
        ControlType velocity_control_type = ControlType::None;
        ControlType linear_full_state_feedback_control_type = ControlType::None;
        ControlType nonlinear_control_type = ControlType::None;

        AxialControlLaw axial_control_law;
        VelocityControlLaw velocity_control_law;
        LinearFullStateFeedbackControlLaw linear_full_state_feedback_control_law;
        NonlinearControlLaw nonlinear_control_law;

        ControlOutput step(const ControlLawInput& ctrl_law_input, bool trim_bool);
    };

    Eigen::VectorXd unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs& u_surface);
    Eigen::VectorXd unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs& u_propulsor);
    Eigen::VectorXd unpack_actuator_inputs(const SurfaceActuatorInputs& u_surface, const PropulsorActuatorInputs& u_propulsor);
}
