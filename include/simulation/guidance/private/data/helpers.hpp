#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	/**
	 * @brief Packs a guidance vector into a structured guidance setpoint.
	 *
	 * The input vector is ordered as body translational velocity, body angular
	 * velocity, roll, pitch, and yaw. Its velocity components use [m/s] and
	 * [rad/s], and its Euler-angle components use [rad].
	 *
	 * @param[in] guidance_vec Guidance vector to pack.
	 * @return Structured translational-velocity, angular-velocity, and Euler-angle setpoint.
	 */
	GuidanceSetpoint pack_guidance_setpoint(const GuidanceSetpointVector& guidance_vec);

} // namespace guidance
