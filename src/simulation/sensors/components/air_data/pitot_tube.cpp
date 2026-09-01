#include "simulation/sensors/public/components/air_data/pitot_tube.hpp"

namespace sensors {

    StagnationAirPressureMeasurement PitotTube::measure(const atmospheric::StagnationAirPressure& P0, double dt) {
        return { step(P0.data, prev_P0_lag, dt) };
    }

}
