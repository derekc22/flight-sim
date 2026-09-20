#pragma once
#include "simulation/linearization/public/data/types.hpp"

#include <string>

namespace linearization
{

	/**
	 * @brief Formats the state and input Jacobians of a local linearization.
	 *
	 * @param[in] lin Local linearization to format.
	 * @return Text containing the labeled A and B matrices.
	 */
	std::string print_linearization_solution(const LocalLinearization& lin);

} // namespace linearization
