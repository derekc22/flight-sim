#pragma once
#include <optional>
#include "simulation/avionics/public/components/collection.hpp"
#include "simulation/avionics/public/data/types.hpp"

namespace avionics {

    struct AvionicsManager {
        Avionics avionics;
        std::optional<AvionicsMeasurements> hist;

        AvionicsManagerOutput step(const AvionicsManagerInput& input);
        AvionicsMeasurements step(
            const sensors::SensorMeasurements& sensor_meas,
            const std::optional<sensors::SensorMeasurements> sensor_hist,
            const sensors::SensorGroundTruth& sensor_gt,
            const AvionicsGroundTruth& avionics_gt,
            double dt
        );

        Settings settings;
    };

}
