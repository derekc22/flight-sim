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

    struct LinearQuadraticRegulatorInput : LinearFullStateFeedbackControlLawInput {
        dynamics::RigidBodyState& zN_t;
        trim::TrimActuatorInputs<double>& u_sol_trim;
        const linearization::TrimStateJacobian& A;
        const linearization::TrimInputJacobian& B;
        guidance::LinearFullStateFeedbackSetpoint setpoint;
    };

    struct LinearQuadraticRegulatorParameters : LinearQuadraticControlLawParameters, LinearFullStateFeedbackControlLawParameters {};

    struct LinearQuadraticRegulator {
        LinearQuadraticRegulatorParameters params;
        LinearQuadraticControlLaw policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        ControlOutput step(const LinearQuadraticRegulatorInput& ctrl_law_input);

        virtual LinearQuadraticControlLawInput make_linear_quadratic_control_law_input(
            const LinearQuadraticRegulatorInput& ctrl_law_input
        );
        trim::TrimStateVector_T<double> unpack_linear_quadratic_regulator_setpoint(
            const guidance::LinearFullStateFeedbackSetpoint& linear_quadratic_regulator_setpoint
        );
    };
}
