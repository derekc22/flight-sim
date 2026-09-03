#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion {

	template <typename T>
	PropulsionState_T<T> make_propulsion_state_T(
	    const actuators::PropulsorActuators& propulsors,
	    const PropellerOmegaStateSet_T<T>& propeller_state)
	{
		return {.front_propulsor_omega = propulsors.front_propulsor.propellers.has_value()
		        ? std::optional<T>{propeller_state.front_propulsor.omega}
		        : std::optional<T>{},
		    .left_propulsor_omega = propulsors.left_propulsor.propellers.has_value()
		        ? std::optional<T>{propeller_state.left_propulsor.omega}
		        : std::optional<T>{},
		    .right_propulsor_omega = propulsors.right_propulsor.propellers.has_value()
		        ? std::optional<T>{propeller_state.right_propulsor.omega}
		        : std::optional<T>{}};
	}

} // namespace propulsion
