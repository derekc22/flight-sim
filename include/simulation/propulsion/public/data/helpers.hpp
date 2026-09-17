#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	/**
	 * @brief Builds the input for one propulsor effector from manager input.
	 *
	 * Copies the center-of-gravity position, body angular velocity, air density, time step, and steady-state flag,
	 * then associates them with @p thrust.
	 *
	 * @tparam T Scalar type used for the propulsion calculation.
	 * @param[in] input State, atmosphere, actuator inputs, and timing data supplied to the propulsion manager.
	 * @param[in] thrust Propulsor thrust [N].
	 * @return Input data for one propulsor effector.
	 */
	template <typename T>
	PropulsorEffectorInput_T<T> make_propulsor_effector_input_T(const PropulsionManagerInput_T<T>& input,
		const T& thrust);

} // namespace propulsion

#include "simulation/propulsion/public/data/helpers.tpp"
