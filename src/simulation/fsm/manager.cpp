#include <stdexcept>
#include <spdlog/spdlog.h>
#include "simulation/fsm/private/data/helpers.hpp"
#include "simulation/fsm/public/manager.hpp"

namespace fsm {

	FSMManager::FSMManager(
	    const Settings& settings)
	{
		if (settings.trim_enabled) {
			autopilot_mode = FiniteState::AutopilotTrim;
		}
		if (settings.control_enabled) {
			autopilot_mode = FiniteState::Autopilot;
		}

		if (settings.joystick_enabled) {
			current_mode = FiniteState::Manual;
		} else {
			current_mode = autopilot_mode;
		}
	}

	FSMManagerOutput FSMManager::step(
	    const FSMManagerInput& input)
	{
		FiniteState prev_mode = current_mode;

		if (input.mode_toggled) {
			switch (current_mode) {
				case FiniteState::Manual: {
					current_mode = autopilot_mode;
					break;
				}

				case FiniteState::AutopilotTrim:
				case FiniteState::Autopilot:
				case FiniteState::None: {
					current_mode = FiniteState::Manual;
					break;
				}

				default:
					throw std::runtime_error("fsm::FSMManager::step Unknown current mode");
			}

			spdlog::info("fsm::FSMManager::step: Switch from mode {} -> {}",
			    mode_to_string(prev_mode),
			    mode_to_string(current_mode));
		}

		return {.current_mode = current_mode};
	}

} // namespace fsm
