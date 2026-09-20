#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct EstimationWrapper {
		dynamics::RigidBodyState Zt_1;

		/**
		 * @brief Produces the estimated rigid-body state for one simulation step.
		 *
		 * @param[in,out] input Aircraft estimator, scheduler, and simulation data.
		 * @return Estimated rigid-body state.
		 */
		EstimationWrapperOutput step(const EstimationWrapperInput& input);
	};

} // namespace runner
