#include "simulation/sensors/public/components/navigation/gnss.hpp"

namespace sensors {

    PositionMeasurement GNSSReceiver::measure(const dynamics::Position& pI_BI, double dt) {
        return { step(pI_BI.data, prev_pI_BI_lag, dt) };
    }

    TranslationalVelocityMeasurement GNSSReceiver::measure(const dynamics::TranslationalVelocity& vB_BI, double dt) {
        return { step(vB_BI.data, prev_vB_BI_lag, dt) };
    }

}
