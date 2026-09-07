#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	PropulsionState make_propulsion_state(
		const PropulsorEffectors& propulsor_effectors,
		const PropellerOmegaStateSet_T<double>& propeller_omega_state_set)
	{
		return make_propulsion_state_T<double>(propulsor_effectors, propeller_omega_state_set);
	}

} // namespace propulsion
