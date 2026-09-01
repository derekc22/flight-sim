#pragma once
#include <Eigen/Dense>
#include "simulation/control/private/policies/linear_quadratic/policy.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct LinearQuadraticRegulator {
        LinearQuadraticPolicy policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        VirtualControlOutput_T<double> step(const FullStateControlInput& input, double);

        LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const FullStateControlInput& input);
    };
}
