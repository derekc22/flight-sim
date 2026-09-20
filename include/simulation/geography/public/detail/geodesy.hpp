#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace frames
{
	struct Frame;
}

namespace geography
{

	/**
	 * @brief Computes the spherical-Earth geographic state of a frame.
	 *
	 * @param[in] F Frame whose geographic state is computed.
	 * @param[in] E ECEF reference frame.
	 * @return Geocentric latitude [rad], longitude [rad], and geometric altitude [m].
	 */
	GeographicState compute_geographic_state(const frames::Frame& F, const frames::Frame& E);

	/**
	 * @brief Constructs the ECEF-to-NED orientation matrix from latitude and longitude.
	 *
	 * @param[in] lat Geocentric latitude [rad].
	 * @param[in] lon Longitude [rad].
	 * @return ECEF-to-NED orientation matrix CEN [-].
	 */
	dynamics::OrientationMatrix CEN_from_lat_lon(const Latitude& lat, const Longitude& lon);

	/**
	 * @brief Converts spherical geographic coordinates to an ECEF position.
	 *
	 * @param[in] geo Geocentric latitude [rad], longitude [rad], and geometric altitude [m].
	 * @return Position relative to Earth center expressed in ECEF axes [m].
	 */
	dynamics::Position pE_from_lat_lon_alt(const GeographicState& geo);

} // namespace geography
