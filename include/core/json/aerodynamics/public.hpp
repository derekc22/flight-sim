#pragma once
#include "simulation/aerodynamics/public.hpp"
#include "simulation/structural/public.hpp"

namespace json {

    aerodynamics::AerodynamicProperties parse_aerodynamics_config();

}
