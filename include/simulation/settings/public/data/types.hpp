#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/avionics/public/data/types.hpp"

namespace settings {

	struct Settings {

		actuators::Settings actuator_settings;
		avionics::Settings avionics_settings;
	};

} // namespace settings
