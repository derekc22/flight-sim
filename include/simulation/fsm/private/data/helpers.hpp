#pragma once
#include "simulation/fsm/public/data/types.hpp"

#include <string_view>

namespace fsm
{

	/**
	 * @brief Returns the display name of a finite-state-machine mode.
	 *
	 * @param[in] mode Mode to convert.
	 * @return Static string naming @p mode.
	 */
	std::string_view mode_to_string(FiniteState mode);

} // namespace fsm
