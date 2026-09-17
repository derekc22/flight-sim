#pragma once
#include "simulation/qp/public/data/types.hpp"

#include <cstddef>

namespace qp
{

	/**
	 * @brief Validates a quadratic-program decision dimension.
	 *
	 * @param[in] dimension Number of decision variables.
	 * @return The validated dimension.
	 */
	std::size_t validate_dimension(std::size_t dimension);

	/**
	 * @brief Validates a quadratic problem against the solver dimension.
	 *
	 * Requires a square symmetric Hessian, matching gradient and bound sizes, finite objective coefficients, non-NaN
	 * bounds, and lower bounds no greater than upper bounds. Infinite bounds are permitted.
	 *
	 * @param[in] problem Quadratic problem to validate.
	 * @param[in] dimension Expected number of decision variables.
	 */
	void validate_problem(const Problem& problem, std::size_t dimension);

} // namespace qp
