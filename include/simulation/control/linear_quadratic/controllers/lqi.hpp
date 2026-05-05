#pragma once
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"

namespace control { struct ControlOutput; } // forward declare

namespace control {

    struct LinearQuadraticIntegratorParameters : LinearQuadraticRegulatorParameters {
        Eigen::MatrixXd Qi;
    };

    struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
        LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params);

        LinearQuadraticControllerInput make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& ctrl_law_input) override;
    };

}