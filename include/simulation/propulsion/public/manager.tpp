#pragma once
#include "simulation/propulsion/public/data/helpers.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace propulsion
{

	template <typename T>
	PropulsionManagerOutput_T<T> PropulsionManager::step(
		const PropulsionManagerInput_T<T>& input)
	{
		const PropulsorEffectorOutput_T<T> front_output = propulsor_effectors.front_propulsor.step<T>(
			make_propulsor_effector_input_T<T>(input, input.u.front_propulsor_cmd));
		const PropulsorEffectorOutput_T<T> left_output = propulsor_effectors.left_propulsor.step<T>(
			make_propulsor_effector_input_T<T>(input, input.u.left_propulsor_cmd));
		const PropulsorEffectorOutput_T<T> right_output = propulsor_effectors.right_propulsor.step<T>(
			make_propulsor_effector_input_T<T>(input, input.u.right_propulsor_cmd));

		dynamics::Wrench_T<T> WB_propulsive;
		WB_propulsive.F = front_output.WB_propulsive.F + left_output.WB_propulsive.F + right_output.WB_propulsive.F;
		WB_propulsive.M = front_output.WB_propulsive.M + left_output.WB_propulsive.M + right_output.WB_propulsive.M;

		return {
			.WB_propulsive = WB_propulsive,
			.propulsion_state_t = {
				.front_propulsor_omega = front_output.propeller_omega,
				.left_propulsor_omega = left_output.propeller_omega,
				.right_propulsor_omega = right_output.propeller_omega
			}
		};
	}

} // namespace propulsion
