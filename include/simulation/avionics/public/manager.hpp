#pragma once
#include <optional>
#include "simulation/avionics/public/components/collection.hpp"
#include "simulation/avionics/public/data/types.hpp"

namespace avionics {

    struct AvionicsManager {
        Avionics avionics;
        std::optional<AvionicsMeasurements> hist;

        AvionicsManagerOutput step(const AvionicsManagerInput& input);

        Settings settings;
    };

}
