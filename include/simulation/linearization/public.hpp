#pragma once
#include <Eigen/Dense>
#include <string>
#include "simulation/constants/public.hpp"
#include "simulation/operating/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace linearization {

    using StateJacobian = Eigen::Matrix<double, constants::state_dim, constants::state_dim>;
    using InputJacobian = Eigen::Matrix<double, constants::state_dim, constants::input_dim>;

    using OutputJacobian = Eigen::Matrix<double, constants::state_dim, constants::state_dim>;
    using DirectFeedthroughJacobian = Eigen::Matrix<double, constants::state_dim, constants::input_dim>;

    struct LocalLinearization {
        StateJacobian A = StateJacobian::Zero();
        InputJacobian B = InputJacobian::Zero();
        OutputJacobian C = OutputJacobian::Identity();
        DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
    };

    struct DiscretizedLocalLinearization {
        StateJacobian A = StateJacobian::Zero();
        InputJacobian B = InputJacobian::Zero();
        OutputJacobian C = OutputJacobian::Identity();
        DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
    };

    DiscretizedLocalLinearization discretize(const linearization::LocalLinearization& lin_sol);
    DiscretizedLocalLinearization discretize_euler(const linearization::LocalLinearization& lin_sol);
    LocalLinearization linearize_operating_point(vehicles::Aircraft& aircraft, const operating::OperatingPoint& operating_point, const operating::OperatingConditions& conditions);
    std::string print_linearization_solution(const LocalLinearization& lin);
}
