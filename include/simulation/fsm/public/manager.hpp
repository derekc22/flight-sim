#pragma once
#include "simulation/fsm/public/data/types.hpp"

namespace fsm {

	struct FSMManager {
		FiniteState autopilot_mode = FiniteState::None;
		FiniteState current_mode;
		
		FSMManager(const Settings& settings);

		FSMManagerOutput step(const FSMManagerInput& input);
	};

}
