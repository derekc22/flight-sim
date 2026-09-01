#include "simulation/sensors/public/components/inertial/gyroscope.hpp"

namespace sensors {

    AngularVelocityMeasurement Gyroscope::measure(const dynamics::AngularVelocity& wB_BI, double dt) {
        return { step(wB_BI.data, prev_wB_BI_lag, dt) };
    }

}
