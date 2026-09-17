#pragma once
#include "simulation/trim/public/data/types.hpp"

#include <string>

namespace trim
{

	/**
	 * @brief Formats a trim solution as a human-readable report.
	 *
	 * The report includes convergence metrics, state, aerodynamic quantities, actuator inputs, wrench, and raw and
	 * weighted residuals.
	 *
	 * @param[in] trim_sol Trim solution to format.
	 * @return Multi-section textual report.
	 */
	std::string print_trim_solution(const TrimSolution& trim_sol);

} // namespace trim
