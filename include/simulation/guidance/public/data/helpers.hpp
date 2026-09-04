#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance
{

	GuidanceSetpointVector unpack_guidance_setpoint(const dynamics::RigidBodyState& Xt);

}
