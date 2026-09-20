#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct TrackingGuidance {
		Trajectory trajectory;
		int k = 0;

		/**
		 * @brief Returns the next tracking setpoint.
		 *
		 * @return Setpoint packed from the current trajectory row.
		 */
		GuidanceSetpoint step();
	};

} // namespace guidance
