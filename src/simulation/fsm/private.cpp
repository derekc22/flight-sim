#include <stdexcept>
#include <string>
#include "simulation/fsm/public.hpp"

namespace fsm {

	std::string_view mode_to_string(FiniteState mode) {
		switch (mode) {
			case FiniteState::ManualMode: 
				return "Manual";
			case FiniteState::AutopilotTrimMode: 
				return "AutopilotTrim";
			case FiniteState::AutopilotMode: 
				return "Autopilot";
			case FiniteState::None: 
				return "None";

			default:
				throw std::runtime_error("fsm::FiniteStateMachine::step Unknown mode");
		}
	}

}