#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace frames {
	struct Frame;
}

namespace geography {

	GeographicState compute_geographic_state(const frames::Frame& F, const frames::Frame& E);

	dynamics::OrientationMatrix CEN_from_lat_lon(const Latitude& lat, const Longitude& lon);

	dynamics::Position pE_from_lat_lon_alt(const GeographicState& geo);

} // namespace geography
