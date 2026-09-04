#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct RegulationGuidance {
		Trajectory trajectory;

		GuidanceSetpoint step();
	};

} // namespace guidance
