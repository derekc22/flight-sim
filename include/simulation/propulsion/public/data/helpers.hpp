#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	template <typename T>
	PropulsionState_T<T> make_propulsion_state_T(const actuators::PropulsorActuators& propulsor_actuators,
		const PropellerOmegaStateSet_T<T>& propeller_omega_state_set);

	PropulsionState make_propulsion_state(const actuators::PropulsorActuators& propulsor_actuators,
		const PropellerOmegaStateSet_T<double>& propeller_omega_state_set);

} // namespace propulsion

#include "simulation/propulsion/public/data/helpers.tpp"
