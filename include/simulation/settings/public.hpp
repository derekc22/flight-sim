#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/vehicles/public.hpp"

namespace settings {



	struct SettingsManager {

		actuators::Settings actuator_settings;
		avionics::Settings avionics_settings;

		vehicles::Aircraft& populate(vehicles::Aircraft& aircraft);

	};


}