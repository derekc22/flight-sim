#include <stdexcept>
#include <spdlog/spdlog.h>
#include "simulation/runner/public.hpp"
#include "simulation/fsm/public.hpp"
#include "simulation/fsm/private.hpp"

namespace fsm {

	FiniteStateMachine::FiniteStateMachine(const runner::JSONFlags& json_flags)
	{
		if (json_flags.trim_flag) {
			autopilot_mode = FiniteState::AutopilotTrim;
		}
		if (json_flags.control_flag) {
			autopilot_mode = FiniteState::Autopilot;
		}

		if (json_flags.joystick_flag) {	
			current_mode = FiniteState::Manual;
		}
		else {
			current_mode = autopilot_mode;
		}

	}

	FiniteState FiniteStateMachine::step(bool mode_toggled) {
		FiniteState prev_mode = current_mode;

		if (mode_toggled) {
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
					throw std::runtime_error("fsm::FiniteStateMachine::step Unknown current mode");
			}

			spdlog::info("fsm::FiniteStateMachine::step: Switch from mode {} -> {}", 
				mode_to_string(prev_mode),
				mode_to_string(current_mode)
			);
		}

		return current_mode;
	}

}
