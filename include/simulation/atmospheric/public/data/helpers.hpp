#pragma once
#include "simulation/atmospheric/public/data/types.hpp"

namespace atmospheric
{

	/**
	 * @brief Constructs a horizontal NED wind-velocity vector from heading and speed.
	 *
	 * The heading is the direction from which the wind blows. The returned down component is zero.
	 *
	 * @param[in] heading_deg Wind heading [deg].
	 * @param[in] spd_kts Wind speed [kt].
	 * @return Wind velocity expressed in NED axes [m/s].
	 */
	Wind build_wind(double heading_deg, double spd_kts);

} // namespace atmospheric
