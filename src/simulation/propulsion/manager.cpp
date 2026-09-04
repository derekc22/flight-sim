#include "simulation/propulsion/public/manager.hpp"

#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"
#include "simulation/propulsion/public/detail/state.hpp"

namespace propulsion
{

	PropulsionManagerOutput PropulsionManager::step(
		const PropulsionManagerInput& input)
	{
		const PropellerOmegaStateSet_T<double> propeller_state_t = compute_propeller_omega_state_set(
			input.propulsors, prev_state, input.u, input.atm.rho, input.dt, input.steady_state);

		return {
			.WB_propulsive = compute_propulsive_loads(
				input.propulsors, input.pB_GB, input.twist, input.atm, input.u, propeller_state_t),
			.state_t = make_propulsion_state(input.propulsors, propeller_state_t)
		};
	}

	void PropulsionManager::commit(
		const PropulsionState& state_t)
	{
		// only the runtime/non-autodiff path should enter this branch because
		// 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
		// 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
		prev_state = state_t;
	}

} // namespace propulsion
