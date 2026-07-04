#include <stdexcept>
#include <spdlog/spdlog.h>
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/guidance/private.hpp"
#include "simulation/trim/public.hpp"

namespace guidance {

    GuidanceSetpointVector unpack_guidance_setpoint(const dynamics::RigidBodyState& Xt) {
        dynamics::EulerAngles eulIB;
        eulIB.set(Xt.q);

        GuidanceSetpointVector out;
        out.head<constants::state_dim>() = dynamics::unpack_state(Xt);
        out(constants::state_dim) = eulIB.psi();

        return out;
    }

    GuidanceSetpoint GuidanceProperties::step(int kf) {
        GuidanceSetpoint out;
        switch (trajectory_type) {
            case TrajectoryType::Regulation: {
                out = pack_guidance_setpoint(trajectory.data.row(0).transpose());
            }
            break;

            case TrajectoryType::Tracking: {
                if (k >= trajectory.data.rows()) { 
                    k = trajectory.data.rows() - 1; 
                    spdlog::warn("GuidanceProperties: Tracking trajectory was fully consumed. Reusing last setpoint");     
                }
                out = pack_guidance_setpoint(trajectory.data.row(k).transpose());
            }
            break;

            case TrajectoryType::Interpolated: {
                if (kf <= 1) { 
                    throw std::runtime_error("GuidanceProperties::step: kf <= 1 for interpolated trajectory"); 
                }
                GuidanceSetpointVector setpoint_t = ((trajectory.data.row(1) - trajectory.data.row(0)) * 
                                                    (static_cast<double>(k) / (kf - 1)) + 
                                                    trajectory.data.row(0)).transpose();
                out = pack_guidance_setpoint(setpoint_t);
            }
            break;

            default:
                throw std::runtime_error("control::step invalid trajectory type");
        }
        ++k;
        return out;
    }

}
