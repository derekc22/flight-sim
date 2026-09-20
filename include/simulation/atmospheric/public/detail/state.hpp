#pragma once
#include "simulation/atmospheric/public/data/types.hpp"

namespace frames
{
	struct Frame;
}

namespace atmospheric
{

	/**
	 * @brief Computes the static atmospheric state at a frame's geometric altitude.
	 *
	 * @param[in] F Frame at which the atmospheric state is evaluated.
	 * @param[in] E ECEF reference frame used to determine geographic altitude.
	 * @return Static air temperature [K], pressure [Pa], density [kg/m^3], and dynamic viscosity [kg/(m s)].
	 */
	StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F, const frames::Frame& E);

} // namespace atmospheric
