#pragma once
#include <tuple>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/control/shared/public.hpp"
#include "simulation/control/pid/public.hpp"

namespace control {

    struct VelocityPIDParameters : PIDPolicyParameters {};

    struct VelocityPID {
        PIDPolicy policy;

        VelocityPID(const VelocityPIDParameters& params);
        ControlOutput step(const VelocityControllerInput& input, double dt);

        PIDPolicyInput make_pid_policy_input(const VelocityControllerInput& input);
    };
}
