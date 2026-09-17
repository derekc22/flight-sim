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

		/**
		 * @brief Solves a bound-constrained quadratic program.
		 *
		 * Solves an objective of the form `0.5 * x^T H x + g^T x` subject to componentwise lower and upper bounds.
		 *
		 * @param[in] problem Hessian, gradient, and decision-variable bounds.
		 * @return Decision vector and mapped solver status.
		 */
		Solution solve(const Problem& problem);
	};

} // namespace qp
