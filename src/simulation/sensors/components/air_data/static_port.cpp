#include "simulation/sensors/public/components/air_data/static_port.hpp"

namespace sensors {

    StaticAirPressureMeasurement StaticPort::measure(const atmospheric::StaticAirPressure& P, double dt) {
        return { step(P.data, prev_P_lag, dt) };
    }

}
