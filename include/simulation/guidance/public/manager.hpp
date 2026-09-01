#pragma once
#include <optional>
#include "simulation/guidance/public/components/interpolated.hpp"
#include "simulation/guidance/public/components/regulation.hpp"
#include "simulation/guidance/public/components/tracking.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace guidance {

    struct GuidanceManager {
        std::optional<RegulationGuidance> regulation;
        std::optional<TrackingGuidance> tracking;
        std::optional<InterpolatedGuidance> interpolated;

        GuidanceManagerOutput step(const GuidanceManagerInput& input);
        GuidanceSetpoint step(int kf);
    };

}
