#include "simulation/sensors/public/components/air_data/aoa_vane.hpp"

namespace sensors {

    AngleOfAttackMeasurement AngleOfAttackVane::measure(const aerodynamics::AngleOfAttack& alpha, double dt) {
        return { step(alpha.data, prev_alpha_lag, dt) };
    }

}
