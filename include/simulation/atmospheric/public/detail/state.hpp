#pragma once
#include "simulation/atmospheric/public/data/types.hpp"

namespace frames {
	struct Frame;
}

namespace atmospheric {

	StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F, const frames::Frame& E);

}
