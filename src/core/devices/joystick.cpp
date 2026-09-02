#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <SDL3/SDL.h>
#include "core/devices/public/joystick.hpp"
#include "simulation/constants/public.hpp"

namespace devices {

    static double normalize_axis(Sint16 value) {
        return value >= 0 ? static_cast<double>(value) / SDL_JOYSTICK_AXIS_MAX : static_cast<double>(value) / -SDL_JOYSTICK_AXIS_MIN;
    }

    static double normalize_trigger(Sint16 value) {
        return std::clamp(static_cast<double>(value) / SDL_JOYSTICK_AXIS_MAX, 0.0, 1.0);
    }

    static double apply_deadzone(double value) {
        double magnitude = std::abs(value);
        if (magnitude <= joystick_deadzone) { return 0.0; }
        return std::copysign((magnitude - joystick_deadzone) / (1.0 - joystick_deadzone), value);
    }

    static double button_rate(SDL_Gamepad* gamepad, SDL_GamepadButton increase, SDL_GamepadButton decrease) {
        return static_cast<double>(SDL_GetGamepadButton(gamepad, increase)) - static_cast<double>(SDL_GetGamepadButton(gamepad, decrease));
    }

    static double map_axis(double input, double limit_min, double limit_max) {
        return input >= 0.0 ? input * limit_max : -input * limit_min;
    }

    static double step_propulsor(double previous, double rate, double limit_min, double limit_max) {
        double delta = rate * (limit_max - limit_min) * joystick_propulsor_rate * constants::dt;
        return std::clamp(previous + delta, limit_min, limit_max);
    }

    JoystickManager::JoystickManager(const actuators::ActuatorLimits& actuator_limits) :
        actuator_limits(actuator_limits)
    {
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
        if (!SDL_Init(SDL_INIT_GAMEPAD)) { throw std::runtime_error(std::string("devices::JoystickManager: Failed to initialize SDL gamepad support: ") + SDL_GetError()); }

        int count = 0;
        SDL_JoystickID* gamepads = SDL_GetGamepads(&count);
        bool dualshock_found = false;
        for (int i = 0; i < count; ++i) {
            if (SDL_GetGamepadTypeForID(gamepads[i]) != SDL_GAMEPAD_TYPE_PS4) { continue; }
            dualshock_found = true;
            gamepad = SDL_OpenGamepad(gamepads[i]);
            if (gamepad) { break; }
        }
        SDL_free(gamepads);

        if (!gamepad) {
            std::string message = dualshock_found
                ? std::string("devices::JoystickManager: Failed to open DualShock 4: ") + SDL_GetError()
                : "devices::JoystickManager: DualShock 4 not found";
            SDL_QuitSubSystem(SDL_INIT_GAMEPAD);
            throw std::runtime_error(message);
        }

        SDL_UpdateGamepads();
        prev_touchpad_down = SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_TOUCHPAD);
    }

    JoystickManager::~JoystickManager() {
        if (gamepad) { SDL_CloseGamepad(gamepad); }
        SDL_QuitSubSystem(SDL_INIT_GAMEPAD);
    }

    JoystickOutputRaw JoystickManager::poll() {
        SDL_PumpEvents();
        SDL_UpdateGamepads();
        if (!SDL_GamepadConnected(gamepad)) { 
            throw std::runtime_error("devices::JoystickManager: DualShock 4 disconnected"); 
        }

        bool touchpad_down = SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_TOUCHPAD);
        if (touchpad_down && !prev_touchpad_down) {
            mode_toggled = true;
        }
        prev_touchpad_down = touchpad_down;

        JoystickOutputRaw raw_output {
            .elevator = apply_deadzone(normalize_axis(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTY))),
            .aileron = apply_deadzone(normalize_axis(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX))),
            .rudder = apply_deadzone(normalize_axis(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX))),
            .left_propulsor_rate = button_rate(gamepad, SDL_GAMEPAD_BUTTON_DPAD_UP, SDL_GAMEPAD_BUTTON_DPAD_DOWN),
            .right_propulsor_rate = button_rate(gamepad, SDL_GAMEPAD_BUTTON_NORTH, SDL_GAMEPAD_BUTTON_SOUTH),
            .front_propulsor_rate = normalize_trigger(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER)) - normalize_trigger(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER))
        };
        return raw_output;
    }

    JoystickManagerOutput JoystickManager::step(const JoystickManagerInput& input) {
        JoystickOutputRaw raw_output = poll();
        const actuators::ActuatorInputs_T<double>& limit_min = actuator_limits.limit_min;
        const actuators::ActuatorInputs_T<double>& limit_max = actuator_limits.limit_max;
        control::ControlOutput u_cmd = input.u_cmd_t_1;

        u_cmd.surface_inputs.elevator_cmd = map_axis(
            raw_output.elevator, 
            limit_min.surface_inputs.elevator_cmd, 
            limit_max.surface_inputs.elevator_cmd
        );
        u_cmd.surface_inputs.aileron_cmd = map_axis(
            raw_output.aileron, 
            limit_min.surface_inputs.aileron_cmd, 
            limit_max.surface_inputs.aileron_cmd
        );
        u_cmd.surface_inputs.rudder_cmd = map_axis(
            raw_output.rudder, 
            limit_min.surface_inputs.rudder_cmd, 
            limit_max.surface_inputs.rudder_cmd
        );

        u_cmd.propulsor_inputs.front_propulsor_cmd = step_propulsor(
            input.u_cmd_t_1.propulsor_inputs.front_propulsor_cmd,
            raw_output.front_propulsor_rate, 
            limit_min.propulsor_inputs.front_propulsor_cmd, 
            limit_max.propulsor_inputs.front_propulsor_cmd
        );
        u_cmd.propulsor_inputs.left_propulsor_cmd = step_propulsor(
            input.u_cmd_t_1.propulsor_inputs.left_propulsor_cmd,
            raw_output.left_propulsor_rate, 
            limit_min.propulsor_inputs.left_propulsor_cmd, 
            limit_max.propulsor_inputs.left_propulsor_cmd
        );
        u_cmd.propulsor_inputs.right_propulsor_cmd = step_propulsor(
            input.u_cmd_t_1.propulsor_inputs.right_propulsor_cmd,
            raw_output.right_propulsor_rate, 
            limit_min.propulsor_inputs.right_propulsor_cmd, 
            limit_max.propulsor_inputs.right_propulsor_cmd
        );

        JoystickManagerOutput joystick_output{ .u_cmd = u_cmd, .mode_toggled = mode_toggled };
        mode_toggled = false;

        return joystick_output;
    }

}
