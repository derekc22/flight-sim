#include "simulation/vehicles/public.hpp"
#include "simulation/settings/public.hpp"

namespace settings {

	vehicles::Aircraft& SettingsManager::populate(vehicles::Aircraft& aircraft) {
		aircraft.actuator_properties.settings = actuator_settings;
		aircraft.avionics_properties.settings = avionics_settings;
		return aircraft;
	}

}