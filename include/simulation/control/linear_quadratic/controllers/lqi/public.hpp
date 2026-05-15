#pragma once
#include <Eigen/Dense>
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"

namespace control {

    struct LinearQuadraticIntegratorParameters : LinearQuadraticRegulatorParameters {
        Eigen::MatrixXd Qi;
    };

    struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
        LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params);

        LinearQuadraticControllerInput make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input) override;
    };

}