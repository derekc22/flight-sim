#pragma once
#include "simulation/atmospheric/public.hpp"
#include "simulation/geography/public.hpp"

namespace atmospheric {

    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude);

    StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M);

    StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M);

}
