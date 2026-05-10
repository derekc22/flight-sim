#pragma once
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"

namespace linearization {

    using StateJacobian = Eigen::Matrix<double, constants::state_dim, constants::state_dim>;
    using TrimInputJacobian = Eigen::Matrix<double, constants::state_dim, constants::input_dim>;

    struct TrimLinearization {
        StateJacobian A = StateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    struct DiscretizedTrimLinearization {
        StateJacobian A = StateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    DiscretizedTrimLinearization discretize(const linearization::TrimLinearization& lin_sol);
}
