#pragma once
#include "simulation/control/private/data/types.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct LinearQuadraticPolicy {
        LinearQuadraticPolicyParameters params;

        LinearQuadraticPolicy(const LinearQuadraticPolicyParameters& params);
        VirtualControlOutputVector_T<double> step(const LinearQuadraticPolicyInput& input);
    };
}
