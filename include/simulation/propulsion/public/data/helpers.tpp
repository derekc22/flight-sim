#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	template <typename T>
	PropulsionState_T<T> make_propulsion_state_T(
		const PropulsorEffectors& propulsor_effectors,
		const PropellerOmegaStateSet_T<T>& propeller_omega_state_set)
	{
		return {
			.front_propulsor_omega = propulsor_effectors.front_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.front_propulsor.omega}
				: std::optional<T>{},
			.left_propulsor_omega = propulsor_effectors.left_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.left_propulsor.omega}
				: std::optional<T>{},
			.right_propulsor_omega = propulsor_effectors.right_propulsor.propellers.has_value()
				? std::optional<T>{propeller_omega_state_set.right_propulsor.omega}
				: std::optional<T>{}
		};
	}

} // namespace propulsion
