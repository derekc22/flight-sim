#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	/**
	 * @brief Unpacks a rigid-body state into a guidance setpoint vector.
	 *
	 * The returned vector is ordered as body translational velocity, body angular
	 * velocity, roll, pitch, and yaw. Position is not included. Its velocity
	 * components use [m/s] and [rad/s], and its Euler-angle components use [rad].
	 *
	 * @param[in] Xt Rigid-body state to unpack.
	 * @return Guidance setpoint vector for the supplied state.
	 */
	GuidanceSetpointVector unpack_guidance_setpoint(const dynamics::RigidBodyState& Xt);

} // namespace guidance
