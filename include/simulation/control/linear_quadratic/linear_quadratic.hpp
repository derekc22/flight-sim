#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/types/types.hpp"

namespace control {

    struct LinearQuadraticControllerInput {
        types::StateVector_T<double> zN_t;
        types::StateVector_T<double> zN_t_des;
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
        types::ActuatorInputsVector_T<double> step(const LinearQuadraticControllerInput& controller_input);
    };
}
