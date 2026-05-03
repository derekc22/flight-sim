
#include "simulation/control/pid/controllers/axial.hpp"


namespace control {

    AxialPID::AxialPID(const AxialPIDParamters& params) :
        lateral_policy( PIDControlLaw({ 
                .Kp = params.Kp_lateral, 
                .Ki = params.Ki_lateral, 
                .Kd = params.Kd_lateral, 
                .tau = params.tau_lateral
            })
        ),
        longitudinal_policy( PIDControlLaw({ 
                .Kp = params.Kp_longitudinal, 
                .Ki = params.Ki_longitudinal, 
                .Kd = params.Kd_longitudinal, 
                .tau = params.tau_longitudinal
            })
        ),
        vertical_policy( PIDControlLaw({ 
                .Kp = params.Kp_vertical, 
                .Ki = params.Ki_vertical, 
                .Kd = params.Kd_vertical, 
                .tau = params.tau_vertical
            })
        )
    {};

    PIDControlLawInput AxialPID::make_pid_control_law_input(
        const AxialPIDInput& ctrl_law_input,
        ControlAxis control_axis
    ){
        dynamics::RigidBodyState zN_t = ctrl_law_input.zN_t;
        actuators::SurfaceActuators surface_actuators = ctrl_law_input.surface_actuators;
        AxialPIDSetpoint setpoint = ctrl_law_input.setpoint;

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
                throw std::runtime_error("control::make_pid_control_law_input invalid control axis");
        }
    }

    ControlOutput AxialPID::step(const AxialPIDInput& ctrl_law_input){
        SurfaceActuatorInputs u_surface{};
        PropulsorActuatorInputs u_propulsor{};

        u_surface.aileron_cmd = lateral_policy.step(
            make_pid_control_law_input(ctrl_law_input, ControlAxis::Lateral)
        );

        u_surface.elevator_cmd = longitudinal_policy.step(
            make_pid_control_law_input(ctrl_law_input, ControlAxis::Longitudinal)
        );

        u_surface.rudder_cmd = vertical_policy.step(
            make_pid_control_law_input(ctrl_law_input, ControlAxis::Vertical)
        );

        return { u_surface, u_propulsor };
    }

}