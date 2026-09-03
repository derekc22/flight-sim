#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace propulsion {

	template <typename T>
	PropulsionManagerOutput_T<T> PropulsionManager::step_T(
	    const PropulsionManagerInput_T<T>& input)
	{
		const PropellerOmegaStateSet_T<T> propeller_state_t = compute_propeller_omega_state_set_T<T>(
		    input.propulsors, prev_state, input.u, input.atm.rho, input.dt, input.steady_state);

		return {.WB_propulsive = compute_propulsive_loads_T<T>(
		            input.propulsors, input.pB_GB, input.twist, input.atm, input.u, propeller_state_t),
		    .state_t = make_propulsion_state_T<T>(input.propulsors, propeller_state_t)};
	}

} // namespace propulsion
