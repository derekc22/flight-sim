#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace atmospheric
{

	StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M);

	MachNumber mps_to_mach(const dynamics::TranslationalVelocity& v, const StaticAirTemperature& T);

	StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& atm, const MachNumber& M);

	MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P);

} // namespace atmospheric
