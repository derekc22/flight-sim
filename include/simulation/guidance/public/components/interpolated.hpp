#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct InterpolatedGuidance {
		Trajectory trajectory;
		int k = 0;

		GuidanceSetpoint step(int kf);
	};

} // namespace guidance
