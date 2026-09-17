#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace atmospheric
{

	/**
	 * @brief Computes static air temperature from stagnation temperature and Mach number.
	 *
	 * @param[in] T0 Stagnation air temperature [K].
	 * @param[in] M Mach number [-].
	 * @return Static air temperature [K].
	 */
	StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M);

	/**
	 * @brief Computes Mach number from translational velocity and static air temperature.
	 *
	 * @param[in] v Translational velocity [m/s].
	 * @param[in] T Static air temperature [K].
	 * @return Ratio of velocity magnitude to the local speed of sound [-].
	 */
	MachNumber mps_to_mach(const dynamics::TranslationalVelocity& v, const StaticAirTemperature& T);

	/**
	 * @brief Converts a static atmospheric state to a stagnation atmospheric state.
	 *
	 * Converts temperature and pressure using @p M while copying density and dynamic viscosity unchanged.
	 *
	 * @param[in] atm Static atmospheric state.
	 * @param[in] M Mach number [-].
	 * @return Stagnation atmospheric state.
	 */
	StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& atm, const MachNumber& M);

	/**
	 * @brief Computes Mach number from stagnation and static air pressures.
	 *
	 * @param[in] P0 Stagnation air pressure [Pa].
	 * @param[in] P Static air pressure [Pa].
	 * @return Mach number [-].
	 */
	MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P);

} // namespace atmospheric
