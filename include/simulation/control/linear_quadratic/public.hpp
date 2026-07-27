#pragma once
#include <optional>
#include <Eigen/Dense>
#include "simulation/control/shared/public.hpp"

namespace control {

    struct LinearQuadraticPolicyInput {
        Eigen::VectorXd zt;
        Eigen::MatrixXd A_virtual;
        constants::MatrixX_T<double, Eigen::Dynamic, constants::virtual_input_dim> B_virtual;
    };

    struct LinearQuadraticPolicyParameters {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;
    };

    struct LinearQuadraticPolicy {
        LinearQuadraticPolicyParameters params;

        LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params);
        VirtualControlOutputVector_T<double> step(const LinearQuadraticPolicyInput& input);
    };
}
