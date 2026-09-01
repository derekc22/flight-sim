#pragma once
#include <cstddef>
#include <proxsuite/proxqp/dense/dense.hpp>
#include "simulation/qp/public/data/types.hpp"

namespace qp {

    struct Solver {
        std::size_t dimension;
        proxsuite::proxqp::dense::QP<double> solver;

        Solver(std::size_t dimension);
        Solution solve(const Problem& problem);
    };

}
