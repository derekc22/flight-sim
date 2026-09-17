#pragma once
#include "simulation/runner/public/data/types.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	/**
	 * @brief Loads and initializes an aircraft from the configured simulation data.
	 *
	 * Subsystem configuration, initial frame state, and aircraft settings are
	 * parsed and applied before the aircraft is returned.
	 *
	 * @param[in] aircraft_id Aircraft identifier.
	 * @param[in] json_flags Configuration-dependent simulation flags.
	 * @return Initialized aircraft.
	 */
	vehicles::Aircraft load_vehicle(const std::string& aircraft_id, const JSONFlags& json_flags);

} // namespace runner
