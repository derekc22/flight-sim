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
		 * Advances through the trajectory one row per call. After the trajectory is
		 * consumed, the final setpoint is reused and a warning is logged.
		 *
		 * @return Setpoint packed from the current trajectory row.
		 */
		GuidanceSetpoint step();
	};

} // namespace guidance
