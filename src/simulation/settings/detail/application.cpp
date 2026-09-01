#include "simulation/settings/public/detail/application.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace settings {

	vehicles::Aircraft& apply_settings(vehicles::Aircraft& aircraft, const Settings& settings) {
		aircraft.actuator_manager.settings = settings.actuator_settings;
		aircraft.avionics_manager.settings = settings.avionics_settings;
		return aircraft;
	}

}
