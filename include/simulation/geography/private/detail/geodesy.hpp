#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace geography
{

	/**
	 * @brief Converts an ECEF position to spherical geographic coordinates.
	 *
	 * @param[in] pE Position relative to Earth center expressed in ECEF axes [m].
	 * @return Geocentric latitude [rad], longitude [rad], and geometric altitude [m].
	 */
	GeographicState lat_lon_alt_from_pE(const dynamics::Position& pE);

} // namespace geography
