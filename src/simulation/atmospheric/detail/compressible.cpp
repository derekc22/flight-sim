#include "simulation/atmospheric/private/detail/compressible.hpp"

#include "simulation/atmospheric/public/detail/compressible.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/math.hpp"

#include <cmath>

namespace atmospheric
{

	StaticAirTemperature T_from_T0(
		const StagnationAirTemperature& T0,
		const MachNumber& M)
	{
		double T = T0.data / (1 + ((constants::gamma_air - 1) / 2) * M.data * M.data);
		return {T};
	};

	MachNumber mps_to_mach(
		const dynamics::TranslationalVelocity& v,
		const StaticAirTemperature& T)
	{
		double a = util::sqrt(constants::gamma_air * constants::R_air * T.data);
		double M = v.data.norm() / a;
		return {M};
	}

	StagnationAtmosphericState static_to_stagnation(
		const StaticAtmosphericState& atm,
		const MachNumber& M)
	{
		StagnationAirTemperature T0 = T0_from_T(atm.T, M);
		StagnationAirPressure P0 = P0_from_P(atm.P, M);

		return {T0, P0, atm.rho, atm.mu};
	};

	MachNumber compute_mach(
		const StagnationAirPressure& P0,
		const StaticAirPressure& P)
	{
		double M = util::sqrt((2.0 / (constants::gamma_air - 1.0)) *
			(std::pow(P0.data / P.data, (constants::gamma_air - 1.0) / constants::gamma_air) - 1.0));
		return {M};
	}

} // namespace atmospheric
