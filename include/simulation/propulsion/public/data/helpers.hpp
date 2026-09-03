#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion {

	template <typename T>
	PropulsionState_T<T> make_propulsion_state_T(const actuators::PropulsorActuators& propulsors,
	    const PropellerOmegaStateSet_T<T>& propeller_state);

	PropulsionState make_propulsion_state(const actuators::PropulsorActuators& propulsors,
	    const PropellerOmegaStateSet_T<double>& propeller_state);

} // namespace propulsion

#include "simulation/propulsion/public/data/helpers.tpp"
