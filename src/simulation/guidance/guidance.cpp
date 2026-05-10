#include <stdexcept>
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/guidance/guidance.hpp"
#include "simulation/trim/shared.hpp"

namespace guidance {

    GuidanceStateVector unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t){
        dynamics::EulerAngles eulIB;
        eulIB.set(xN_t.q);

        GuidanceStateVector out;
        out.head<constants::state_dim>() = trim::unpack_rigid_body_state(xN_t);
        out(constants::state_dim) = eulIB.psi();

        return out;
    }

    GuidanceSetpoint pack_guidance_setpoint(const GuidanceStateVector& guidance_vec) {
        dynamics::LinearVelocity v{ guidance_vec.segment<3>(0) };
        dynamics::AngularVelocity w{ guidance_vec.segment<3>(3) };
        dynamics::EulerAngles eul{ guidance_vec.segment<3>(6).reverse() };

        return { .vB_BI = v, .wB_BI = w, .eulIB = eul };
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
                GuidanceStateVector setpoint_t = ((trajectory.data.row(1) - trajectory.data.row(0)) * (static_cast<double>(t) / (tf - 1)) + trajectory.data.row(0)).transpose();
                return pack_guidance_setpoint(setpoint_t);
            }
            break;

            default:
                throw std::runtime_error("control::step invalid trajectory type");
        }
    }

}
