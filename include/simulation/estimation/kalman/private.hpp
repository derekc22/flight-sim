#pragma once
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    dynamics::RigidBodyState make_kalman_state_estimate(const dynamics::RigidBodyState& Yt, const dynamics::StateVector& zt);

}
