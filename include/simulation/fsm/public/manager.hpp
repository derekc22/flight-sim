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
		 * A toggle from manual selects @c autopilot_mode. A toggle from either
		 * autopilot mode or from @c None selects manual. Successful transitions are
		 * logged; without a toggle, the current mode is unchanged.
		 *
		 * @param[in] input Mode-toggle command.
		 * @return Current mode after applying the command.
		 */
		FSMManagerOutput step(const FSMManagerInput& input);
	};

} // namespace fsm
