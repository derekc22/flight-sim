#pragma once
#include "simulation/atmospheric/public/data/types.hpp"

namespace atmospheric
{

	StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M);

	StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M);

	StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M);

	StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& atm0, const MachNumber& M);

} // namespace atmospheric
