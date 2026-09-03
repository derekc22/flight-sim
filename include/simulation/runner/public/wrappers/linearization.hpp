#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

	struct LinearizationWrapper {
		linearization::LocalLinearization lin_sol;

		// initialize virtual linearization solution
		linearization::VirtualLocalLinearization virtual_lin_sol;
		analysis::EigenAnalysis eig_sol;

		LinearizationWrapperOutput step(const LinearizationWrapperInput& input);
	};

} // namespace runner
