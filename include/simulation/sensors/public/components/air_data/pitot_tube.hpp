#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

    struct PitotTube : Sensor {
        std::optional<double> prev_P0_lag;
        StagnationAirPressureMeasurement measure(const atmospheric::StagnationAirPressure& P0, double dt);
    };

}
