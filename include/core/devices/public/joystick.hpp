#pragma once
#include "core/devices/public/data/types.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/actuators/public/data/helpers.hpp"

struct SDL_Gamepad;

namespace devices {

    struct JoystickManager {
        SDL_Gamepad* gamepad = nullptr;
        bool prev_touchpad_down = false;
        bool mode_toggled = false;
        actuators::ActuatorLimits actuator_limits;

        JoystickManager(const actuators::ActuatorLimits& actuator_limits);
        ~JoystickManager();
        JoystickManager(const JoystickManager&) = delete;
        JoystickManager& operator=(const JoystickManager&) = delete;

        JoystickOutputRaw poll();
        JoystickOutput step(const control::ControlOutput& u_cmd_t_1);
    };

}
