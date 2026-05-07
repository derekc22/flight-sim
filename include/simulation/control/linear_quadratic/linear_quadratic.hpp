#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/linearization.hpp"

namespace control {

    struct LinearQuadraticControllerInput {
        trim::TrimStateVector_T<double> meas;
        trim::TrimStateVector_T<double> meas_des;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
    };

    struct LinearQuadraticControllerParameters {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;
        bool integrator_bool = false;
    };

    struct LinearQuadraticController {
        LinearQuadraticControllerParameters params;

        LinearQuadraticController(const LinearQuadraticControllerParameters& params);
        Eigen::VectorXd step(const LinearQuadraticControllerInput& controller_input);
    };
}