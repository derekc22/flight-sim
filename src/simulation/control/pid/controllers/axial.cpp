
#include "simulation/control/pid/controllers/axial.hpp"


namespace control {

    AxialPID::AxialPID(const AxialPIDParameters& params) :
        params(params),
        lateral_policy({
                .Kp = params.Kp_roll,
                .Ki = params.Ki_roll,
                .Kd = params.Kd_roll,
                .tau = params.tau
            }
        ),
        longitudinal_policy({
                .Kp = params.Kp_pitch,
                .Ki = params.Ki_pitch,
                .Kd = params.Kd_pitch,
                .tau = params.tau
            }
        ),
        vertical_policy({
                .Kp = params.Kp_yaw,
                .Ki = params.Ki_yaw,
                .Kd = params.Kd_yaw,
                .tau = params.tau
            }
        )
    {};

    PIDControllerInput AxialPID::make_pid_controller_input(const AxialControllerInput& controller_input, ControlAxis control_axis){
        dynamics::RigidBodyState zN_t = controller_input.zN_t;
        actuators::SurfaceActuators surface_actuators = controller_input.surface_actuators;
        guidance::AxialSetpoint setpoint = controller_input.setpoint;

        dynamics::EulerAngles eul_meas_t;
        eul_meas_t.set(zN_t.q);

        switch (control_axis) {
            case ControlAxis::Lateral:
                return {
                    .meas = eul_meas_t.phi(),
                    .meas_des = setpoint.eulIB.phi(),
                    .meas_dot = zN_t.w.p(),
                    .limit_min = surface_actuators.aileron.limit_min,
                    .limit_max = surface_actuators.aileron.limit_max
                };

            case ControlAxis::Longitudinal:
                return {
                    .meas = eul_meas_t.theta(),
                    .meas_des = setpoint.eulIB.theta(),
                    .meas_dot = zN_t.w.q(),
                    .limit_min = surface_actuators.elevator.limit_min,
                    .limit_max = surface_actuators.elevator.limit_max
                };

            case ControlAxis::Vertical:
                return {
                    .meas = eul_meas_t.psi(),
                    .meas_des = setpoint.eulIB.psi(),
                    .meas_dot = zN_t.w.r(),
                    .limit_min = surface_actuators.rudder.limit_min,
                    .limit_max = surface_actuators.rudder.limit_max
                };

            default:
                throw std::runtime_error("control::make_pid_controller_input invalid control axis");
        }
    }

    ControlOutput AxialPID::step(const AxialControllerInput& controller_input){
        SurfaceActuatorInputs u_surface{};
        PropulsorActuatorInputs u_propulsor{};

        u_surface.aileron_cmd = lateral_policy.step(
            make_pid_controller_input(controller_input, ControlAxis::Lateral)
        );

        u_surface.elevator_cmd = longitudinal_policy.step(
            make_pid_controller_input(controller_input, ControlAxis::Longitudinal)
        );

        u_surface.rudder_cmd = vertical_policy.step(
            make_pid_controller_input(controller_input, ControlAxis::Vertical)
        );

        return { u_surface, u_propulsor };
    }

}
