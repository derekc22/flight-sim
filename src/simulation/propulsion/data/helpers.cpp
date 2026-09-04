#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	PropulsionState make_propulsion_state(
		const actuators::PropulsorActuators& propulsor_actuators,
		const PropellerOmegaStateSet_T<double>& propeller_omega_state_set)
	{
		return make_propulsion_state_T<double>(propulsor_actuators, propeller_omega_state_set);
	}

} // namespace propulsion
