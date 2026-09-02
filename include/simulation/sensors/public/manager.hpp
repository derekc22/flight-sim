#pragma once
#include <optional>
#include "simulation/sensors/public/components/collection.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

    struct SensorManager {
        Sensors sensors;
        std::optional<SensorMeasurements> hist;

        SensorManagerOutput step(const SensorManagerInput& input);
    };

}
