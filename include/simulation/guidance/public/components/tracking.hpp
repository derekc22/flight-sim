#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct TrackingGuidance {
		Trajectory trajectory;
		int k = 0;

		GuidanceSetpoint step();
	};

} // namespace guidance
