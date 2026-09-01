#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

    struct Magnetometer : Sensor {
        std::optional<double> prev_heading_lag;
        HeadingMeasurement measure(const geography::Heading& heading, double dt);
    };

}
