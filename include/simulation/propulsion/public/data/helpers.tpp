#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	template <typename T>
	PropulsionState_T<T> make_propulsion_state_T(
		const actuators::PropulsorActuators& propulsor_actuators,
		const PropellerOmegaStateSet_T<T>& propeller_omega_state_set)
	{
		return {
			.front_propulsor_omega = propulsor_actuators.front_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.front_propulsor.omega}
				: std::optional<T>{},
			.left_propulsor_omega = propulsor_actuators.left_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.left_propulsor.omega}
				: std::optional<T>{},
			.right_propulsor_omega = propulsor_actuators.right_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.right_propulsor.omega}
				: std::optional<T>{}
		};
	}

} // namespace propulsion
