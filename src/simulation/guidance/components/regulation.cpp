#include "simulation/guidance/public/components/regulation.hpp"

#include "simulation/guidance/private/data/helpers.hpp"

namespace guidance
{

	GuidanceSetpoint RegulationGuidance::step()
	{
		return pack_guidance_setpoint(trajectory.data.row(0).transpose());
	}

} // namespace guidance
