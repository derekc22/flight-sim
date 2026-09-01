#include <stdexcept>
#include "simulation/guidance/private/data/helpers.hpp"
#include "simulation/guidance/public/components/interpolated.hpp"

namespace guidance {

    GuidanceSetpoint InterpolatedGuidance::step(int kf) {
        if (kf <= 1) { 
            throw std::runtime_error("GuidanceManager::step: kf <= 1 for interpolated trajectory");
        }
        GuidanceSetpointVector setpoint_t = ((trajectory.data.row(1) - trajectory.data.row(0)) * 
                                            (static_cast<double>(k) / (kf - 1)) + 
                                            trajectory.data.row(0)).transpose();
        GuidanceSetpoint out = pack_guidance_setpoint(setpoint_t);
        ++k;
        return out;
    }

}
