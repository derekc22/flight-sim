#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	template <typename T>
	PropulsorEffectorInput_T<T> make_propulsor_effector_input_T(const PropulsionManagerInput_T<T>& input,
		const T& thrust);

} // namespace propulsion

#include "simulation/propulsion/public/data/helpers.tpp"
