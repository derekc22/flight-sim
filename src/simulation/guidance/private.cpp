#include "simulation/guidance/private.hpp"
#include "simulation/dynamics/public.hpp"

namespace guidance {

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceStateVector& guidance_vec) {
        dynamics::TranslationalVelocity v{ guidance_vec.segment<3>(0) };
        dynamics::AngularVelocity w{ guidance_vec.segment<3>(3) };
        dynamics::EulerAngles eul{ guidance_vec.segment<3>(6).reverse() };

        return { .vB_BI = v, .wB_BI = w, .eulIB = eul };
    }

}
