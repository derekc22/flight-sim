#pragma once
#include <Eigen/Dense>
#include "simulation/control/shared/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"

namespace control {

    struct LinearQuadraticRegulatorParameters : LinearQuadraticPolicyParameters {};

    struct LinearQuadraticRegulator {
        LinearQuadraticPolicy policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        ControlOutput step(const LinearQuadraticControllerInput& input, double dt);

        LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input);
    };
}
