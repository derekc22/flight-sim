#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/linear_quadratic/linear_quadratic.hpp"
#include "simulation/guidance/guidance.hpp"

namespace control { struct ControlOutput; struct LinearFullStateFeedbackControllerInput; } // forward declare

namespace control {

    struct LinearQuadraticRegulatorParameters : LinearQuadraticControllerParameters, LinearFullStateFeedbackControllerParameters {};

    struct LinearQuadraticRegulator {
        LinearQuadraticRegulatorParameters params;
        LinearQuadraticController policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        ControlOutput step(const LinearFullStateFeedbackControllerInput& controller_input);

        virtual LinearQuadraticControllerInput make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input);
        trim::TrimStateVector_T<double> unpack_linear_quadratic_regulator_setpoint(const guidance::LinearFullStateFeedbackSetpoint& linear_quadratic_regulator_setpoint);
    };
}
