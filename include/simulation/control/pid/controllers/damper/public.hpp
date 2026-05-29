
#pragma once

#include "simulation/control/pid/controllers/axial/public.hpp"

namespace control {

    struct DamperPID : AxialPID {
        using AxialPID::AxialPID;

        PIDPolicyInput make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis) override;
    };
}
