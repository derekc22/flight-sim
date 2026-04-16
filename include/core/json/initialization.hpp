#pragma once
#include "simulation/vehicles/vehicles.hpp"

namespace json {

    vehicles::StepOptions parse_initialization_config(bool trim_enabled);

}
