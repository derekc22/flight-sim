#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace atmospheric
{

	/**
	 * @brief Computes a simplified standard atmosphere at a geometric altitude.
	 *
	 * Uses a constant lapse-rate region through 11 km and an isothermal region above 11 km.
	 *
	 * @param[in] altitude Geometric altitude [m].
	 * @return Static air temperature [K], pressure [Pa], density [kg/m^3], and dynamic viscosity [kg/(m s)].
	 */
	StaticAtmosphericState std_atmosphere(const geography::GeometricAltitude& altitude);

} // namespace atmospheric
