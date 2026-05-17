#pragma once
#include "simulation/guidance/public.hpp"

namespace guidance {

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceStateVector& guidance_vec);

}
