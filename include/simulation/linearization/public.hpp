#pragma once
#include <Eigen/Dense>
#include <string>
#include "simulation/constants/public.hpp"
#include "simulation/trim/public.hpp"

namespace vehicles { struct Aircraft; }

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
    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const trim::TrimSolution& trim_sol);
    std::string print_linearization_solution(const TrimLinearization& lin);
}
