#pragma once
#include "simulation/settings/public/data/types.hpp"

namespace vehicles
{
	struct Aircraft;
}

namespace settings
{

	/**
	 * @brief Applies simulation settings to an aircraft's subsystem managers.
	 *
	 * Copies actuator settings into the actuator manager and avionics settings
	 * into the avionics manager.
	 *
	 * @param[in,out] aircraft Aircraft whose subsystem settings are updated.
	 * @param[in] settings Settings to apply.
	 * @return Reference to the updated @p aircraft.
	 */
	vehicles::Aircraft& apply_settings(vehicles::Aircraft& aircraft, const Settings& settings);

} // namespace settings
