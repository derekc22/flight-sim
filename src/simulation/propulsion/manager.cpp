#include "simulation/propulsion/public/manager.hpp"

#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"

namespace propulsion
{

	PropulsionManagerOutput PropulsionManager::step(
		const PropulsionManagerInput& input)
	{
		const PropellerOmegaStateSet_T<double> propeller_omega_state_set = compute_propeller_omega_state_set(
			input.propulsor_actuators, propulsion_state_t_1, input.u, input.atm.rho, input.dt, input.steady_state);

		return {
			.WB_propulsive = compute_propulsive_loads(
				input.propulsor_actuators, input.pB_GB, input.twist, input.atm, input.u, propeller_omega_state_set),
			.propulsion_state_t = make_propulsion_state(input.propulsor_actuators, propeller_omega_state_set)
		};
	}

	void PropulsionManager::commit(
		const PropulsionState& propulsion_state_t)
	{
		// only the runtime/non-autodiff path should enter this branch because
		// 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
		// 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
		propulsion_state_t_1 = propulsion_state_t;
	}

} // namespace propulsion
