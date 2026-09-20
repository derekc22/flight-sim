#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct TrimWrapper {
		trim::TrimSolution trim_sol;

		/**
		 * @brief Computes and stores the aircraft trim solution.
		 *
		 * @param[in] input Aircraft, automatic-differentiation model, and body-frame wind.
		 * @return Computed trim solution.
		 */
		TrimWrapperOutput step(const TrimWrapperInput& input);
	};

} // namespace runner
