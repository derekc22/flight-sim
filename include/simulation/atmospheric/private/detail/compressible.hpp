#pragma once
#include "simulation/atmospheric/public/data/types.hpp"

namespace atmospheric
{

	/**
	 * @brief Computes stagnation air temperature from static temperature and Mach number.
	 *
	 * @param[in] T Static air temperature [K].
	 * @param[in] M Mach number [-].
	 * @return Stagnation air temperature [K].
	 */
	StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M);

	/**
	 * @brief Computes stagnation air pressure from static pressure and Mach number.
	 *
	 * @param[in] P Static air pressure [Pa].
	 * @param[in] M Mach number [-].
	 * @return Stagnation air pressure [Pa].
	 */
	StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M);

	/**
	 * @brief Computes static air pressure from stagnation pressure and Mach number.
	 *
	 * @param[in] P0 Stagnation air pressure [Pa].
	 * @param[in] M Mach number [-].
	 * @return Static air pressure [Pa].
	 */
	StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M);

	/**
	 * @brief Converts a stagnation atmospheric state to a static atmospheric state.
	 *
	 * Converts temperature and pressure using @p M while copying density and dynamic viscosity unchanged.
	 *
	 * @param[in] atm0 Stagnation atmospheric state.
	 * @param[in] M Mach number [-].
	 * @return Static atmospheric state.
	 */
	StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& atm0, const MachNumber& M);

} // namespace atmospheric
