#pragma once
#include "simulation/qp/public/data/types.hpp"

#include <cstddef>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace qp
{

	struct Solver {
		std::size_t dimension;
		proxsuite::proxqp::dense::QP<double> solver;

		Solver(std::size_t dimension);
		Solution solve(const Problem& problem);
	};

} // namespace qp
