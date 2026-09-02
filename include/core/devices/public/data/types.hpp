#pragma once
#include "simulation/control/public/data/types.hpp"

namespace devices {

    inline constexpr double joystick_deadzone = 0.1;
    inline constexpr double joystick_propulsor_rate = 0.5;

    struct JoystickOutputRaw {
        double elevator = 0.0;
        double aileron = 0.0;
        double rudder = 0.0;
        double left_propulsor_rate = 0.0;
        double right_propulsor_rate = 0.0;
        double front_propulsor_rate = 0.0;
    };

    struct JoystickManagerInput {
        const control::ControlOutput& u_cmd_t_1;
    };

    struct JoystickManagerOutput {
        control::ControlOutput u_cmd;
        bool mode_toggled;
    };

}
