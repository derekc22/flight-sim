#pragma once
#include "simulation/failures/public/data/types.hpp"

namespace failures
{

	struct FailureManager {

		/**
		 * @brief Checks whether the aircraft has reached the terrain-collision threshold.
		 *
		 * @param[in] input Current height above ground level [m].
		 * @return Terrain-collision status for the current step.
		 */
		FailureManagerOutput step(const FailureManagerInput& input);
	};

} // namespace failures
