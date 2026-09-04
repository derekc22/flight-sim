#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	PropulsionState make_propulsion_state(
		const actuators::PropulsorActuators& propulsors,
		const PropellerOmegaStateSet_T<double>& propeller_state)
	{
		return make_propulsion_state_T<double>(propulsors, propeller_state);
	}

} // namespace propulsion
