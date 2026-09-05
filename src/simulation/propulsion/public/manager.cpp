#include "simulation/propulsion/public/manager.hpp"

namespace propulsion
{

	void PropulsionManager::commit(
		const PropulsionState& propulsion_state_t)
	{
		// only the runtime/non-autodiff path should enter this branch because
		// 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
		// 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
		propulsion_state_t_1 = propulsion_state_t;
	}

} // namespace propulsion
