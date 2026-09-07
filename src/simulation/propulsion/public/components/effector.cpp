#include "simulation/propulsion/public/components/effector.hpp"

namespace propulsion
{

	void PropulsorEffector::commit(
		const std::optional<double>& propeller_omega_t)
	{
		// only the runtime/non-autodiff path should enter this branch because
		// 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
		// 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
		propeller_omega_t_1 = propeller_omega_t;
	}

} // namespace propulsion
