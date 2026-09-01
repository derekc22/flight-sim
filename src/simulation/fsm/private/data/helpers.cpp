#include <stdexcept>
#include <string_view>
#include "simulation/fsm/private/data/helpers.hpp"

namespace fsm {

	std::string_view mode_to_string(FiniteState mode) {
		switch (mode) {
			case FiniteState::Manual:
				return "Manual";
			case FiniteState::AutopilotTrim:
				return "AutopilotTrim";
			case FiniteState::Autopilot:
				return "Autopilot";
			case FiniteState::None: 
				return "None";

			default:
				throw std::runtime_error("fsm::FSMManager::mode_to_string Unknown mode");
		}
	}

}
