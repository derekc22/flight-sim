#include <algorithm>
#include "simulation/control/private/policies/linear_quadratic/slicot_care.hpp"
#include "simulation/control/private/policies/linear_quadratic/policy.hpp"

namespace control {

    LinearQuadraticPolicy::LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params) : params(params) {}

    VirtualControlOutputVector_T<double> LinearQuadraticPolicy::step(const LinearQuadraticPolicyInput& input) {

        if (!params.K.has_value()) {
            const CareSolution care_sol = solve_care(input.A_virtual, input.B_virtual, params.Q, params.R);
            params.K = lqr_gain(input.B_virtual, params.R, care_sol.P);
        }

        VirtualControlOutputVector_T<double> mu_deviation = -params.K.value() * input.zt;

        return mu_deviation;
    }

}
