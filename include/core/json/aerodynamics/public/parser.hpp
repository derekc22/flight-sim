#pragma once
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/structural/public/data/types.hpp"

namespace json {

    aerodynamics::AerodynamicsManager parse_aerodynamics_config();

}
