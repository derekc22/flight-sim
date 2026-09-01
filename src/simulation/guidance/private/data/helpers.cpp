#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/private/data/helpers.hpp"

namespace guidance {

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceSetpointVector& guidance_vec) {
        dynamics::TranslationalVelocity v{ guidance_vec.segment<3>(0) };
        dynamics::AngularVelocity w{ guidance_vec.segment<3>(3) };
        dynamics::EulerAngles eul{ guidance_vec.segment<3>(6).reverse() };

        return { .vB_BI = v, .wB_BI = w, .eulIB = eul };
    }

}
