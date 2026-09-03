#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim {

	dynamics::RigidBodyState update_state_from_trim(const dynamics::RigidBodyState& Xt,
	    const dynamics::State_T<double>& trim_state);

	void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators,
	    actuators::PropulsorActuators& propulsor_actuators,
	    const TrimSolution& trim_sol);

} // namespace trim
