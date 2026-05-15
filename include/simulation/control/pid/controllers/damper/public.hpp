
#pragma once

#include "simulation/control/pid/controllers/axial/public.hpp"

namespace control {

    struct DamperPID : AxialPID {
        using AxialPID::AxialPID;

        PIDControllerInput make_pid_controller_input(const AxialControllerInput& controller_input, ControlAxis control_axis) override;
    };
}
