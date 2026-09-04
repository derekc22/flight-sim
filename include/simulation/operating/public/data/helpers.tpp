#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/operating/public/data/helpers.hpp"

namespace operating
{

	template <typename T>
	StateInputVector_T<T> unpack_state_input_T(
		const dynamics::State_T<T>& x,
		const actuators::ActuatorInputs_T<T>& u)
	{
		const actuators::SurfaceActuatorInputs_T<T>& surface_inputs = u.surface_inputs;
		const actuators::PropulsorActuatorInputs_T<T>& propulsor_inputs = u.propulsor_inputs;
		StateInputVector_T<T> out;
		out << x.vx, x.vy, x.vz, x.p, x.q, x.r, x.phi, x.theta, surface_inputs.elevator_cmd, surface_inputs.aileron_cmd,
			surface_inputs.rudder_cmd, propulsor_inputs.front_propulsor_cmd, propulsor_inputs.left_propulsor_cmd,
			propulsor_inputs.right_propulsor_cmd;
		return out;
	}

	template <typename T>
	VirtualStateInputVector_T<T> unpack_virtual_state_input_T(
		const dynamics::State_T<T>& x,
		const dynamics::Wrench_T<T>& u)
	{
		VirtualStateInputVector_T<T> out;
		out << x.vx, x.vy, x.vz, x.p, x.q, x.r, x.phi, x.theta, u.F, u.M;
		return out;
	}

	template <typename T>
	std::tuple<dynamics::StateVector_T<T>, actuators::ActuatorInputsVector_T<T>> split_state_input_vector_T(
		const StateInputVector_T<T>& xu)
	{
		return {
			dynamics::StateVector_T<T>(xu.template head<constants::state_dim>()),
			actuators::ActuatorInputsVector_T<T>(xu.template tail<constants::input_dim>())
		};
	}

	template <typename T>
	dynamics::State_T<T> pack_state_T(
		const StateInputVector_T<T>& xu)
	{
		return {
			.vx = xu(0),
			.vy = xu(1),
			.vz = xu(2),
			.p = xu(3),
			.q = xu(4),
			.r = xu(5),
			.phi = xu(6),
			.theta = xu(7),
		};
	}

	template <typename T>
	actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(
		const StateInputVector_T<T>& xu)
	{
		return {
			.surface_inputs =
				{
					.elevator_cmd = xu(8),
					.aileron_cmd = xu(9),
					.rudder_cmd = xu(10),
				},
			.propulsor_inputs = {
				.front_propulsor_cmd = xu(11),
				.left_propulsor_cmd = xu(12),
				.right_propulsor_cmd = xu(13),
			}
		};
	}

	template <typename T>
	dynamics::Wrench_T<T> pack_virtual_inputs_T(
		const VirtualStateInputVector_T<T>& xu)
	{
		return {
			.F = constants::Vector3_T<T>(xu(8), xu(9), xu(10)), .M = constants::Vector3_T<T>(xu(11), xu(12), xu(13))
		};
	}

	template <typename T>
	actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(
		const StateInputVector_T<T>& xu,
		const actuators::FixedActuatorInputs& fixed_actuator_inputs)
	{
		actuators::ActuatorInputs_T<T> u = pack_actuator_inputs_T<T>(xu);
		u.surface_inputs.flap_cmd = T(fixed_actuator_inputs.flap);
		u.surface_inputs.spoiler_cmd = T(fixed_actuator_inputs.spoiler);
		return u;
	}

	template <typename T>
	OperatingPoint_T<T> pack_state_input_T(
		const StateInputVector_T<T>& xu)
	{
		return {pack_state_T(xu), pack_actuator_inputs_T(xu)};
	}

	template <typename T>
	OperatingPoint_T<T> pack_state_input_T(
		const dynamics::State_T<T>& x,
		const actuators::ActuatorInputs_T<T>& u)
	{
		return {
			.state = x,
			.input = u,
		};
	}

	template <typename T>
	VirtualOperatingPoint_T<T> pack_virtual_state_input_T(
		const VirtualStateInputVector_T<T>& xu)
	{
		return {pack_state_T(xu), pack_virtual_inputs_T(xu)};
	}

} // namespace operating
