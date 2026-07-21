#pragma once
#include "simulation/runner/public.hpp"

namespace fsm {

	enum class FiniteState {
		Manual,
		AutopilotTrim,
		Autopilot,
		None
	};

	struct FiniteStateMachine {
		FiniteState autopilot_mode = FiniteState::None;
		FiniteState current_mode;
		
		FiniteStateMachine(const runner::JSONFlags& json_flags);

		FiniteState step(bool mode_toggled);
	};

}

