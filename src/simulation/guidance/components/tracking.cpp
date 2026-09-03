#include <spdlog/spdlog.h>
#include "simulation/guidance/private/data/helpers.hpp"
#include "simulation/guidance/public/components/tracking.hpp"

namespace guidance {

	GuidanceSetpoint TrackingGuidance::step()
	{
		if (k >= trajectory.data.rows()) {
			k = trajectory.data.rows() - 1;
			spdlog::warn("GuidanceManager: Tracking trajectory was fully consumed. Reusing last setpoint");
		}
		GuidanceSetpoint out = pack_guidance_setpoint(trajectory.data.row(k).transpose());
		++k;
		return out;
	}

} // namespace guidance
