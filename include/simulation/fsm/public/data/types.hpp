#pragma once

namespace fsm
{

	enum class FiniteState {
		Manual,
		AutopilotTrim,
		Autopilot,
		None
	};

	struct Settings {
		bool trim_flag;
		bool control_flag;
		bool joystick_flag;
	};

	struct FSMManagerInput {
		bool mode_toggled;
	};

	struct FSMManagerOutput {
		FiniteState current_mode;
	};

} // namespace fsm
