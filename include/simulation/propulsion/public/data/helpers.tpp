#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"

namespace propulsion
{

	template <typename T>
	PropulsorEffectorInput_T<T> make_propulsor_effector_input_T(
		const PropulsionManagerInput_T<T>& input,
		const T& thrust)
	{
		return {
			.pB_GB = input.pB_GB,
			.wB_BI = input.twist.w,
			.rho = input.atm.rho,
			.thrust = thrust,
			.dt = input.dt,
			.steady_state = input.steady_state
		};
	}

} // namespace propulsion
