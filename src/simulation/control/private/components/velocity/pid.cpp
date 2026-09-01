
#include "simulation/control/private/components/velocity/pid.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace control {

    VelocityPID::VelocityPID(const VelocityPIDParameters& params) :
        policy({ .Kp = params.Kp, .Ki = params.Ki, .Kd = params.Kd, .tau = params.tau })
    {};

    PIDPolicyInput VelocityPID::make_pid_policy_input(const VelocityControlInput& input) {
        dynamics::RigidBodyState Zt = input.estimated_state;
        guidance::VelocitySetpoint setpoint = input.setpoint;

        return {
            .x = Zt.v.data(0),
            .x_des = setpoint.vB_BI.data(0),
            .delta_mu = input.previous_control_residual[0]
        };
    }

    VirtualControlOutput_T<double> VelocityPID::step(const VelocityControlInput& input, double dt) {
        VirtualControlOutput_T<double> out{};

        out.F.x() = policy.step(
            make_pid_policy_input(input),
            dt
        );

        return out;
    }
}
