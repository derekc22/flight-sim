#pragma once
#include "simulation/guidance/public.hpp"

namespace guidance {

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceVector& guidance_vec);

}
