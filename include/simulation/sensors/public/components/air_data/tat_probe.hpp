#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

    struct TotalAirTemperatureProbe : Sensor {
        std::optional<double> prev_T0_lag;
        StagnationAirTemperatureMeasurement measure(const atmospheric::StagnationAirTemperature& T0, double dt);
    };

}
