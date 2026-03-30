#pragma once

#include <Eigen/Dense>
#include "simulation/autopilot/autopilot.hpp"

namespace analysis {
    using TrimStateJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_state_dofs>;
    using TrimInputJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_input_dofs>;

    struct TrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    TrimLinearization linearize_trim_dynamics(const autopilot::TrimState<double>& x, const autopilot::TrimInput<double>& u, const autopilot::TrimModel& model, const autopilot::TrimConditions& conditions);

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const autopilot::TrimSolution& trim);

    std::string print_linerization_solution(const TrimLinearization& lin);

}
