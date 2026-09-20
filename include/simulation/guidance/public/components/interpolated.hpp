#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	struct InterpolatedGuidance {
		Trajectory trajectory;
		int k = 0;

		/**
		 * @brief Returns the next linearly interpolated setpoint.
		 *
		 * @param[in] kf Number of setpoints spanning the interpolation, including both endpoints.
		 * @return Setpoint packed from the interpolated trajectory row.
		 */
		GuidanceSetpoint step(int kf);
	};

} // namespace guidance
