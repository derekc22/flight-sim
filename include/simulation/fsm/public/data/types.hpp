#pragma once

namespace fsm {

	enum class FiniteState { Manual, AutopilotTrim, Autopilot, None };

	struct Settings {
		bool trim_enabled;
		bool control_enabled;
		bool joystick_enabled;
	};

	struct FSMManagerInput {
		bool mode_toggled;
	};

	struct FSMManagerOutput {
		FiniteState current_mode;
	};

} // namespace fsm
