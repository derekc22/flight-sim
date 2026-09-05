#pragma once
#include "core/devices/public/data/types.hpp"
#include "simulation/actuators/public/data/types.hpp"

struct SDL_Gamepad;

namespace devices
{

	struct Joystick {
		SDL_Gamepad* gamepad = nullptr;
		bool prev_touchpad_down = false;
		bool mode_toggled = false;
		actuators::ActuatorLimits actuator_limits;

		Joystick(const actuators::ActuatorLimits& actuator_limits);
		~Joystick();
		Joystick(const Joystick&) = delete;
		Joystick& operator=(const Joystick&) = delete;

		JoystickOutputRaw poll();
		JoystickOutput step(const JoystickInput& input);
	};

} // namespace devices
