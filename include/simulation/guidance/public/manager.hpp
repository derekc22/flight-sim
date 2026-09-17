#pragma once
#include "simulation/guidance/public/components/interpolated.hpp"
#include "simulation/guidance/public/components/regulation.hpp"
#include "simulation/guidance/public/components/tracking.hpp"
#include "simulation/guidance/public/data/types.hpp"

#include <optional>

namespace guidance
{

	struct GuidanceManager {
		std::optional<RegulationGuidance> regulation;
		std::optional<TrackingGuidance> tracking;
		std::optional<InterpolatedGuidance> interpolated;

		/**
		 * @brief Advances the configured guidance component and returns its setpoint.
		 *
		 * If multiple components are configured, regulation takes precedence over
		 * tracking, which takes precedence over interpolated guidance.
		 *
		 * @param[in] input Guidance horizon data used by interpolated guidance.
		 * @return Setpoint produced by the selected guidance component.
		 */
		GuidanceManagerOutput step(const GuidanceManagerInput& input);
	};

} // namespace guidance
