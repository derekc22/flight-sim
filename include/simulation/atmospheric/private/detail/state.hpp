#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace atmospheric
{

	StaticAtmosphericState std_atmosphere(const geography::GeometricAltitude& altitude);

}
