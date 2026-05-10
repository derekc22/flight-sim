#pragma once
#include "simulation/atmospheric/shared.hpp"
#include "simulation/geography/shared.hpp"

namespace atmospheric {

    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude);

    Wind build_wind(double heading_deg, double spd_kts);

}
