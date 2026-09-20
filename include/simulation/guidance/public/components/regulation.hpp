#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct RegulationGuidance {
		Trajectory trajectory;

		/**
		 * @brief Returns the fixed regulation setpoint.
		 *
		 * @return Setpoint packed from the first trajectory row.
		 */
		GuidanceSetpoint step();
	};

} // namespace guidance
