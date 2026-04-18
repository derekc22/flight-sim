#pragma once

#include <Eigen/Dense>
#include "simulation/trim/trim.hpp"

namespace linearization {
    using TrimStateJacobian = Eigen::Matrix<double, trim::trim_state_dofs, trim::trim_state_dofs>;
    using TrimInputJacobian = Eigen::Matrix<double, trim::trim_state_dofs, trim::trim_input_dofs>;

    struct TrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const trim::TrimSolution& trim_sol);

    std::string print_linearization_solution(const TrimLinearization& lin);

}
