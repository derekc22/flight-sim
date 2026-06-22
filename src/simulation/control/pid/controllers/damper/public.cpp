
#include <stdexcept>
#include "simulation/actuators/surface/public.hpp"
#include "simulation/control/pid/controllers/damper/public.hpp"
#include "simulation/dynamics/public.hpp"


namespace control {

    PIDPolicyInput DamperPID::make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis) {
        dynamics::RigidBodyState Zt = input.Zt;
        actuators::SurfaceActuators surface_actuators = input.surface_actuators;

        switch (axis) {
            case ControlAxis::Lateral:
                return {
                    .x = Zt.w.p(),
                    .x_des = input.setpoint.wB_BI.p(),
                    .x_dot = 0.0,
                    .limit_min = surface_actuators.aileron.limit_min,
                    .limit_max = surface_actuators.aileron.limit_max
                };

            case ControlAxis::Longitudinal:
                return {
                    .x = Zt.w.q(),
                    .x_des = input.setpoint.wB_BI.q(),
                    .x_dot = 0.0,
                    .limit_min = surface_actuators.elevator.limit_min,
                    .limit_max = surface_actuators.elevator.limit_max
                };

            case ControlAxis::Vertical:
                return {
                    .x = Zt.w.r(),
                    .x_des = input.setpoint.wB_BI.r(),
                    .x_dot = 0.0,
                    .limit_min = surface_actuators.rudder.limit_min,
                    .limit_max = surface_actuators.rudder.limit_max
                };

            default:
                throw std::runtime_error("control::make_pid_policy_input invalid control axis");
        }
    }

}
