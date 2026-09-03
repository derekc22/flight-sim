#include <cmath>
#include "simulation/atmospheric/private/detail/compressible.hpp"
#include "simulation/atmospheric/public/detail/compressible.hpp"
#include "simulation/constants/public/scalars.hpp"

namespace atmospheric {

	StagnationAirTemperature T0_from_T(
	    const StaticAirTemperature& T,
	    const MachNumber& M)
	{
		double T0 = T.data * (1 + ((constants::gamma_air - 1) / 2) * M.data * M.data);
		return {T0};
	};

	StagnationAirPressure P0_from_P(
	    const StaticAirPressure& P,
	    const MachNumber& M)
	{
		double P0 = P.data *
		    std::pow(1 + ((constants::gamma_air - 1) / 2) * M.data * M.data,
		        constants::gamma_air / (constants::gamma_air - 1));
		return {P0};
	};

	StaticAirPressure P_from_P0(
	    const StagnationAirPressure& P0,
	    const MachNumber& M)
	{
		double P = P0.data /
		    std::pow(1 + ((constants::gamma_air - 1) / 2) * M.data * M.data,
		        constants::gamma_air / (constants::gamma_air - 1));
		return {P};
	};

	StaticAtmosphericState stagnation_to_static(
	    const StagnationAtmosphericState& atm0,
	    const MachNumber& M)
	{
		AirDensity rho = atm0.rho;
		DynamicViscosity mu = atm0.mu;

		StaticAirTemperature T = T_from_T0(atm0.T0, M);
		StaticAirPressure P = P_from_P0(atm0.P0, M);

		return {T, P, rho, mu};
	};

} // namespace atmospheric
