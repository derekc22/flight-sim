#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct LinearizationWrapper {
		linearization::LocalLinearization lin_sol;

		// initialize virtual linearization solution
		linearization::VirtualLocalLinearization virtual_lin_sol;
		analysis::EigenAnalysis eig_sol;

		/**
		 * @brief Computes local linearizations and eigenanalysis about a trim solution.
		 *
		 * @param[in] input Automatic-differentiation model and trim solution.
		 * @return Physical and virtual linearizations with the physical-model eigenanalysis.
		 */
		LinearizationWrapperOutput step(const LinearizationWrapperInput& input);
	};

} // namespace runner
