#pragma once
#include "simulation/settings/public/data/types.hpp"

namespace vehicles {
	struct Aircraft;
}

namespace settings {

	vehicles::Aircraft& apply_settings(vehicles::Aircraft& aircraft, const Settings& settings);

}
