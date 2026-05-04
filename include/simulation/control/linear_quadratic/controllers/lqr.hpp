#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/linear_quadratic/linear_quadratic.hpp"
#include "simulation/guidance/guidance.hpp"

namespace control { struct ControlOutput; struct LinearFullStateFeedbackControlLawInput; } // forward declare

namespace control {

    struct LinearQuadraticRegulatorParameters : LinearQuadraticControlLawParameters, LinearFullStateFeedbackControlLawParameters {};

    struct LinearQuadraticRegulator {
        LinearQuadraticRegulatorParameters params;
        LinearQuadraticControlLaw policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        ControlOutput step(const LinearFullStateFeedbackControlLawInput& ctrl_law_input);

        virtual LinearQuadraticControlLawInput make_linear_quadratic_control_law_input(const LinearFullStateFeedbackControlLawInput& ctrl_law_input);
        trim::TrimStateVector_T<double> unpack_linear_quadratic_regulator_setpoint(const guidance::LinearFullStateFeedbackSetpoint& linear_quadratic_regulator_setpoint);
    };
}
