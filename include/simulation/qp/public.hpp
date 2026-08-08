#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace qp {

    enum class Status {
        Solved,
        MaxIterations,
        PrimalInfeasible,
        DualInfeasible,
        ClosestPrimalFeasible,
        NotRun
    };

    struct Problem {
        Eigen::MatrixXd hessian;
        Eigen::VectorXd gradient;
        Eigen::VectorXd lower;
        Eigen::VectorXd upper;
    };

    struct Solution {
        Eigen::VectorXd x;
        Status status = Status::NotRun;
    };

    struct Solver {
        std::size_t dimension;
        proxsuite::proxqp::dense::QP<double> solver;

        Solver(std::size_t dimension);
        Solution solve(const Problem& problem);
    };

}
