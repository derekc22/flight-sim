#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace propulsion
{

	template <typename T>
	PropulsionManagerOutput_T<T> PropulsionManager::step(
		const PropulsionManagerInput_T<T>& input)
	{
		const PropellerOmegaStateSet_T<T> propeller_omega_state_set = compute_propeller_omega_state_set_T<T>(
			propulsor_effectors, propulsion_state_t_1, input.u, input.atm.rho, input.dt, input.steady_state);

		return {
			.WB_propulsive = compute_propulsive_loads_T<T>(
				propulsor_effectors, input.pB_GB, input.twist, input.atm, input.u, propeller_omega_state_set),
			.propulsion_state_t = make_propulsion_state_T<T>(propulsor_effectors, propeller_omega_state_set)
		};
	}

} // namespace propulsion
