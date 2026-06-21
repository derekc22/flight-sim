#pragma once
#include "simulation/atmospheric/public.hpp"
#include "simulation/geography/public.hpp"

namespace atmospheric {

    StaticAtmosphericState std_atmosphere(const geography::GeometricAltitude& altitude);

    StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M);

    StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M);

    StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M);

    StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& atm0, const MachNumber& M);

}
