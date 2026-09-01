#pragma once
#include "simulation/guidance/public/data/types.hpp"

namespace guidance {

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceSetpointVector& guidance_vec);

}
