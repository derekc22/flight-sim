#include <algorithm>
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/control/linear_quadratic/slicot_care.hpp"
#include "simulation/dynamics/public.hpp"

namespace control {

    LinearQuadraticPolicy::LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params) : params(params) {}

    actuators::ActuatorInputsVector_T<double> LinearQuadraticPolicy::step(const LinearQuadraticPolicyInput& input) {

        if (!params.K.has_value()) {
            const CareSolution care_sol = solve_care(input.A, input.B, params.Q, params.R);
            params.K = lqr_gain(input.B, params.R, care_sol.P);
        }

        actuators::ActuatorInputsVector_T<double> u_deviation;
        u_deviation = -params.K.value() * input.zt;

        return u_deviation;
    }

}
