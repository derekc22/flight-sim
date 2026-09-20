#pragma once
#include "simulation/fsm/public/data/types.hpp"

namespace fsm
{

	struct FSMManager {
		FiniteState autopilot_mode = FiniteState::None;
		FiniteState current_mode;

		FSMManager(const Settings& settings);

		/**
		 * @brief Applies an optional manual/autopilot mode toggle.
		 *
		 * @param[in] input Mode-toggle command.
		 * @return Current mode after applying the command.
		 */
		FSMManagerOutput step(const FSMManagerInput& input);
	};

} // namespace fsm
