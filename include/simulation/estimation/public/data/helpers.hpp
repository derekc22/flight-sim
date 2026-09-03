#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace estimation {

	dynamics::RigidBodyState make_kalman_state_estimate(const dynamics::RigidBodyState& Yt,
	    const dynamics::StateVector_T<double>& zt);

}
