
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
                    .meas = Zt.w.p(),
                    .meas_des = input.setpoint.wB_BI.p(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.aileron.limit_min,
                    .limit_max = surface_actuators.aileron.limit_max
                };

            case ControlAxis::Longitudinal:
                return {
                    .meas = Zt.w.q(),
                    .meas_des = input.setpoint.wB_BI.q(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.elevator.limit_min,
                    .limit_max = surface_actuators.elevator.limit_max
                };

            case ControlAxis::Vertical:
                return {
                    .meas = Zt.w.r(),
                    .meas_des = input.setpoint.wB_BI.r(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.rudder.limit_min,
                    .limit_max = surface_actuators.rudder.limit_max
                };

            default:
                throw std::runtime_error("control::make_pid_policy_input invalid control axis");
        }
    }

}
