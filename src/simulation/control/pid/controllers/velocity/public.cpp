
#include "simulation/control/pid/controllers/velocity/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"

namespace control {

    VelocityPID::VelocityPID(const VelocityPIDParameters& params) :
        policy(params)
    {};

    PIDPolicyInput VelocityPID::make_pid_policy_input(const VelocityControllerInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::VelocitySetpoint setpoint = input.setpoint;

        return {
            .x = Zt.v.data(0),
            .x_des = setpoint.vB_BI.data(0),
            .delta_mu = input.delta_mu_vec_t_1[0]
        };
    }

    VirtualControlOutput_T<double> VelocityPID::step(const VelocityControllerInput& input, double dt) {
        VirtualControlOutput_T<double> out{};

        out.F.x() = policy.step(
            make_pid_policy_input(input),
            dt
        );

        return out;
    }
}
