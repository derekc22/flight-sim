#pragma once
#include <optional>
#include <Eigen/Dense>
#include "simulation/actuators/public.hpp"
// #include "simulation/dynamics/public.hpp"
// #include "simulation/linearization/public.hpp"

namespace control {

    struct LinearQuadraticControllerInput {
        Eigen::VectorXd zN_t;
        Eigen::VectorXd zN_t_des;
        Eigen::MatrixXd A;
        Eigen::Matrix<double, Eigen::Dynamic, constants::input_dim> B;
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
