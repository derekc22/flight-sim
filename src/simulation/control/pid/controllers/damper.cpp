
#include "simulation/control/pid/controllers/damper.hpp"


namespace control {

    PIDControllerInput DamperPID::make_pid_controller_input(const AxialControllerInput& controller_input, ControlAxis control_axis){
        dynamics::RigidBodyState zN_t = controller_input.zN_t;
        actuators::SurfaceActuators surface_actuators = controller_input.surface_actuators;

        switch (control_axis) {
            case ControlAxis::Lateral:
                return {
                    .meas = zN_t.w.p(),
                    .meas_des = controller_input.setpoint.wB_BI.p(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.aileron.limit_min,
                    .limit_max = surface_actuators.aileron.limit_max
                };

            case ControlAxis::Longitudinal:
                return {
                    .meas = zN_t.w.q(),
                    .meas_des = controller_input.setpoint.wB_BI.q(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.elevator.limit_min,
                    .limit_max = surface_actuators.elevator.limit_max
                };

            case ControlAxis::Vertical:
                return {
                    .meas = zN_t.w.r(),
                    .meas_des = controller_input.setpoint.wB_BI.r(),
                    .meas_dot = 0.0,
                    .limit_min = surface_actuators.rudder.limit_min,
                    .limit_max = surface_actuators.rudder.limit_max
                };

            default:
                throw std::runtime_error("control::make_pid_controller_input invalid control axis");
        }
    }

}
