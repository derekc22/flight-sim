#pragma once
#include "simulation/control/public.hpp"
#include "simulation/actuators/public.hpp"

struct SDL_Gamepad;

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

    struct JoystickOutput {
        control::ControlOutput u_cmd;
        bool manual_mode;
    };

    struct JoystickManager {
        SDL_Gamepad* gamepad = nullptr;
        bool prev_touchpad_down = false;
        bool manual_mode = true;
        actuators::ActuatorLimits actuator_limits;

        JoystickManager(const actuators::ActuatorLimits& actuator_limits);
        ~JoystickManager();
        JoystickManager(const JoystickManager&) = delete;
        JoystickManager& operator=(const JoystickManager&) = delete;

        JoystickOutputRaw poll();
        JoystickOutput step(const control::ControlOutput& u_cmd_t_1);
    };

}
