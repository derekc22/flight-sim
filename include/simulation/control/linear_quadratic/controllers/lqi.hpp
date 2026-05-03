#pragma once
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"

namespace control { struct ControlOutput; } // forward declare

namespace control {

    struct LinearQuadraticIntegratorParamters : LinearQuadraticRegulatorParamters {
        Eigen::MatrixXd Qi;
    };

    struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
        LinearQuadraticIntegrator(const LinearQuadraticIntegratorParamters& params);

        LinearQuadraticControlLawInput make_linear_quadratic_control_law_input(
            const LinearQuadraticRegulatorInput& ctrl_law_input
        ) override;
    };

}