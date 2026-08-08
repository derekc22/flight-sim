#include <algorithm>
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/control/linear_quadratic/slicot_care.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/control/shared/public.hpp"

namespace control {

    LinearQuadraticPolicy::LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params) : params(params) {}

    VirtualControlOutputVector_T<double> LinearQuadraticPolicy::step(const LinearQuadraticPolicyInput& input) {

        if (!params.K.has_value()) {
            const CareSolution care_sol = solve_care(input.A_virtual, input.B_virtual, params.Q, params.R);
            params.K = lqr_gain(input.B_virtual, params.R, care_sol.P);
        }

        VirtualControlOutputVector_T<double> mu_deviation;
        mu_deviation = -params.K.value() * input.zt;

        return mu_deviation;
    }

}
