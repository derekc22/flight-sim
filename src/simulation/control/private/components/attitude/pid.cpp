
#include <stdexcept>
#include "simulation/control/private/components/attitude/pid.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"


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

    PIDPolicyInput AttitudePID::make_pid_policy_input(const AttitudeControlInput& input, AttitudeAxis axis) {
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::AttitudeSetpoint setpoint = input.setpoint;

        dynamics::EulerAngles eul_est_t;
        eul_est_t.set(Zt.q);

        switch (axis) {
            case AttitudeAxis::Lateral:
                return {
                    .x = eul_est_t.phi(),
                    .x_des = setpoint.eulIB.phi(),
                    .x_dot = Zt.w.p(),
                    .delta_mu = input.delta_mu_vec_t_1[3]
                };

            case AttitudeAxis::Longitudinal:
                return {
                    .x = eul_est_t.theta(),
                    .x_des = setpoint.eulIB.theta(),
                    .x_dot = Zt.w.q(),
                    .delta_mu = input.delta_mu_vec_t_1[4]
                };

            case AttitudeAxis::Vertical:
                return {
                    .x = eul_est_t.psi(),
                    .x_des = setpoint.eulIB.psi(),
                    .x_dot = Zt.w.r(),
                    .delta_mu = input.delta_mu_vec_t_1[5]
                };

            default:
                throw std::runtime_error("control::make_pid_policy_input invalid control axis");
        }
    }

    VirtualControlOutput_T<double> AttitudePID::step(const AttitudeControlInput& input, double dt) {
        VirtualControlOutput_T<double> out{};

        out.M.x() = lateral_policy.step(
            make_pid_policy_input(input, AttitudeAxis::Lateral),
            dt
        );

        out.M.y() = longitudinal_policy.step(
            make_pid_policy_input(input, AttitudeAxis::Longitudinal),
            dt
        );

        out.M.z() = vertical_policy.step(
            make_pid_policy_input(input, AttitudeAxis::Vertical),
            dt
        );

        return out;
    }

}
