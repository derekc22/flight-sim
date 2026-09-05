#pragma once
#include "simulation/runner/public/data/types.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	vehicles::Aircraft load_vehicle(const std::string& aircraft_id, const JSONFlags& json_flags);

}
