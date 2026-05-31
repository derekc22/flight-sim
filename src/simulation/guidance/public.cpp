#include <stdexcept>
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/guidance/private.hpp"
#include "simulation/trim/public.hpp"

namespace guidance {

    GuidanceSetpointVector unpack_guidance_setpoint(const dynamics::RigidBodyState& xN_t) {
        dynamics::EulerAngles eulIB;
        eulIB.set(xN_t.q);

        GuidanceSetpointVector out;
        out.head<constants::state_dim>() = dynamics::unpack_state(xN_t);
        out(constants::state_dim) = eulIB.psi();

        return out;
    }

    GuidanceSetpoint GuidanceProperties::step(int t, int tf) {
        switch (trajectory_type) {
            case TrajectoryType::Stationary: {
                return pack_guidance_setpoint(trajectory.data.row(0).transpose());
            }
            break;

            case TrajectoryType::Prespecified: {
                if (t >= trajectory.data.rows()) { t = trajectory.data.rows() - 1; }
                return pack_guidance_setpoint(trajectory.data.row(t).transpose());
            }
            break;

            case TrajectoryType::Interpolated: {
                if (tf <= 1) { throw std::runtime_error("GuidanceProperties::step: tf <= 1 for interpolated trajectory"); }
                GuidanceSetpointVector setpoint_t = ((trajectory.data.row(1) - trajectory.data.row(0)) * (static_cast<double>(t) / (tf - 1)) + trajectory.data.row(0)).transpose();
                return pack_guidance_setpoint(setpoint_t);
            }
            break;

            default:
                throw std::runtime_error("control::step invalid trajectory type");
        }
    }

}
