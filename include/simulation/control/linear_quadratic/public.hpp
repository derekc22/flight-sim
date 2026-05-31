#pragma once
#include <optional>
#include <Eigen/Dense>
#include "simulation/actuators/public.hpp"

namespace control {

    struct LinearQuadraticPolicyInput {
        Eigen::VectorXd zt_vec;
        Eigen::MatrixXd A;
        Eigen::Matrix<double, Eigen::Dynamic, constants::input_dim> B;
    };

    struct LinearQuadraticPolicyParameters {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;
    };

    struct LinearQuadraticPolicy {
        LinearQuadraticPolicyParameters params;

        LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params);
        actuators::ActuatorInputsVector_T<double> step(const LinearQuadraticPolicyInput& input);
    };
}
