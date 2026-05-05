#pragma once
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"

namespace linearization {

    using TrimStateJacobian = Eigen::Matrix<double, constants::state_dim, constants::state_dim>;
    using TrimInputJacobian = Eigen::Matrix<double, constants::state_dim, constants::input_dim>;

    struct TrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    struct DiscretizedTrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };
}
