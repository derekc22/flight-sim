#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	/**
	 * @brief Unpacks a rigid-body state into a guidance setpoint vector.
	 *
	 * @param[in] Xt Rigid-body state to unpack.
	 * @return Guidance setpoint vector for the supplied state.
	 */
	GuidanceSetpointVector unpack_guidance_setpoint(const dynamics::RigidBodyState& Xt);

} // namespace guidance
