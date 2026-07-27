#pragma once
#include "simulation/control/shared/public.hpp"
#include "simulation/control/pid/public.hpp"

namespace control {

    struct VelocityPIDParameters : PIDPolicyParameters {};

    struct VelocityPID {
        PIDPolicy policy;

        VelocityPID(const VelocityPIDParameters& params);
        VirtualControlOutput_T<double> step(const VelocityControllerInput& input, double dt);

        PIDPolicyInput make_pid_policy_input(const VelocityControllerInput& input);
    };
}
