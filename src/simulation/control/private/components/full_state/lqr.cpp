#include "simulation/control/private/components/full_state/lqr.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"
#include "simulation/control/private/components/full_state/helpers.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        policy({ .Q = params.Q, .R = params.R, .K = params.K })
    {};

    LinearQuadraticPolicyInput LinearQuadraticRegulator::make_linear_quadratic_policy_input(const FullStateControlInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        dynamics::StateVector_T<double> zt_deviation = dynamics::unpack_state(Zt) - unpack_state(input.setpoint);

        return {
            .zt = zt_deviation,
            .A_virtual = input.virtual_linearization.A_virtual,
            .B_virtual = input.virtual_linearization.B_virtual
        };
    }

    VirtualControlOutput_T<double> LinearQuadraticRegulator::step(const FullStateControlInput& input, double) {
        VirtualControlOutputVector_T<double> mu_deviation = policy.step(
            make_linear_quadratic_policy_input(input)
        );

        return dynamics::pack_wrench_T(mu_deviation);
    }

}
