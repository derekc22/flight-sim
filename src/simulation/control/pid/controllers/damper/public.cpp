
#include <stdexcept>
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/dynamics/public.hpp"


namespace control {

    PIDPolicyInput DamperPID::make_pid_policy_input(const AttitudeControllerInput& input, AttitudeAxis axis) {
        dynamics::RigidBodyState Zt = input.Zt;

        // All 3 attitude axes omit x_dot. Thus, PID policy runs as PID (as opposed to PI-D)
        switch (axis) {
            case AttitudeAxis::Lateral:
                return {
                    .x = Zt.w.p(),
                    .x_des = input.setpoint.wB_BI.p(),
                };

            case AttitudeAxis::Longitudinal:
                return {
                    .x = Zt.w.q(),
                    .x_des = input.setpoint.wB_BI.q(),
                };

            case AttitudeAxis::Vertical:
                return {
                    .x = Zt.w.r(),
                    .x_des = input.setpoint.wB_BI.r(),
                };

            default:
                throw std::runtime_error("control::make_pid_policy_input invalid control axis");
        }
    }

}
