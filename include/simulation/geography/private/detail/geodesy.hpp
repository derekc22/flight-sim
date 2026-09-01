#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace geography {

    GeographicState lat_lon_alt_from_pE(const dynamics::Position & pE);

}
