#include "simulation/vehicles/public.hpp"
#include "simulation/settings/public.hpp"

namespace settings {

	vehicles::Aircraft& SettingsManager::populate(vehicles::Aircraft& aircraft) {
		aircraft.actuator_manager.settings = actuator_settings;
		aircraft.avionics_manager.settings = avionics_settings;
		return aircraft;
	}

}