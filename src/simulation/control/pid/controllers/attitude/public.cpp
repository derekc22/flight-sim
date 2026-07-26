
#include <stdexcept>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/control/pid/controllers/attitude/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"


namespace control {

    AttitudePID::AttitudePID(const AttitudePIDParameters& params) :
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

    PIDPolicyInput AttitudePID::make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis) {
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::AttitudeSetpoint setpoint = input.setpoint;

        dynamics::EulerAngles eul_est_t;
        eul_est_t.set(Zt.q);

        switch (axis) {
            case ControlAxis::Lateral:
                return {
                    .x = eul_est_t.phi(),
                    .x_des = setpoint.eulIB.phi(),
                    .x_dot = Zt.w.p()
                };

            case ControlAxis::Longitudinal:
                return {
                    .x = eul_est_t.theta(),
                    .x_des = setpoint.eulIB.theta(),
                    .x_dot = Zt.w.q()
                };

            case ControlAxis::Vertical:
                return {
                    .x = eul_est_t.psi(),
                    .x_des = setpoint.eulIB.psi(),
                    .x_dot = Zt.w.r()
                };

            default:
                throw std::runtime_error("control::make_pid_policy_input invalid control axis");
        }
    }

    ControlOutput AttitudePID::step(const AttitudeControllerInput& input, double dt) {
        actuators::SurfaceActuatorInputs_T<double> u_surface{};
        actuators::PropulsorActuatorInputs_T<double> u_propulsor{};

        u_surface.aileron_cmd = lateral_policy.step(
            make_pid_policy_input(input, ControlAxis::Lateral),
            dt
        );

        u_surface.elevator_cmd = longitudinal_policy.step(
            make_pid_policy_input(input, ControlAxis::Longitudinal),
            dt
        );

        u_surface.rudder_cmd = vertical_policy.step(
            make_pid_policy_input(input, ControlAxis::Vertical),
            dt
        );

        return { u_surface, u_propulsor };
    }

}
