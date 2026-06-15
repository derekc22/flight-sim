
#pragma once

#include "simulation/control/pid/controllers/axial/public.hpp"

namespace control {

    struct DamperPID : AttitudePID {
        using AttitudePID::AttitudePID;

        PIDPolicyInput make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis) override;
    };
}
