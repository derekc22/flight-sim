#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/control/linear_quadratic/private.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        policy(params)
    {};

    LinearQuadraticPolicyInput LinearQuadraticRegulator::make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::LinearQuadraticSetpoint setpoint = input.setpoint;

        dynamics::StateVector_T<double> zt_deviation = dynamics::unpack_state(Zt) - unpack_state(setpoint);

        return {
            .zt = zt_deviation,
            .A_virtual = input.virtual_linearization.A_virtual,
            .B_virtual = input.virtual_linearization.B_virtual
        };
    }

    VirtualControlOutput_T<double> LinearQuadraticRegulator::step(const LinearQuadraticControllerInput& input, double) {
        VirtualControlOutputVector_T<double> mu_deviation = policy.step(
            make_linear_quadratic_policy_input(input)
        );
        VirtualControlOutputVector_T<double> mu_trim = dynamics::unpack_wrench_T(input.mu_sol_trim);
        VirtualControlOutputVector_T<double> mu = mu_deviation + mu_trim;

        return dynamics::pack_wrench(mu);
    }

}
