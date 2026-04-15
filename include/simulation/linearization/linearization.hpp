#pragma once

#include <Eigen/Dense>
#include "simulation/autopilot/autopilot.hpp"

namespace linearization {
    using TrimStateJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_state_dofs>;
    using TrimInputJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_input_dofs>;

    struct TrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const autopilot::TrimSolution& trim_sol);

    std::string print_linearization_solution(const TrimLinearization& lin);

}
