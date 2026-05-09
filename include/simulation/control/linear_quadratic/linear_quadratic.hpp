#pragma once
#include <optional>
#include <Eigen/Dense>
#include "simulation/actuators/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/linearization/shared.hpp"

namespace control {

    struct LinearQuadraticControllerInput {
        dynamics::StateVector_T<double> zN_t;
        dynamics::StateVector_T<double> zN_t_des;
        linearization::StateJacobian A;
        linearization::TrimInputJacobian B;
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
        actuators::ActuatorInputsVector_T<double> step(const LinearQuadraticControllerInput& controller_input);
    };
}
