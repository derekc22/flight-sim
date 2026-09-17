#pragma once
#include "simulation/operating/public/data/types.hpp"

#include <tuple>

namespace operating
{

	/**
	 * @brief Unpacks a state and actuator inputs into a combined vector.
	 *
	 * The vector order is vx, vy, vz, p, q, r, roll, pitch, elevator, aileron, rudder, front propulsor, left
	 * propulsor, and right propulsor. Flap and spoiler inputs are not included.
	 *
	 * @tparam T Scalar type stored by the state and inputs.
	 * @param[in] x Structured state.
	 * @param[in] u Structured actuator inputs.
	 * @return Combined state-input vector.
	 */
	template <typename T>
	StateInputVector_T<T> unpack_state_input_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

	/**
	 * @brief Unpacks a state and virtual wrench input into a combined vector.
	 *
	 * The vector order is vx, vy, vz, p, q, r, roll, pitch, Fx, Fy, Fz, Mx, My, and Mz.
	 *
	 * @tparam T Scalar type stored by the state and wrench.
	 * @param[in] x Structured state.
	 * @param[in] u Virtual force [N] and moment [N m] input.
	 * @return Combined virtual state-input vector.
	 */
	template <typename T>
	VirtualStateInputVector_T<T> unpack_virtual_state_input_T(const dynamics::State_T<T>& x,
		const dynamics::Wrench_T<T>& u);

	/**
	 * @brief Splits a combined state-input vector into state and actuator-input vectors.
	 *
	 * @tparam T Scalar type stored by the vector.
	 * @param[in] xu Combined state-input vector.
	 * @return State vector followed by actuator-input vector as a tuple.
	 */
	template <typename T>
	std::tuple<dynamics::StateVector_T<T>, actuators::ActuatorInputsVector_T<T>> split_state_input_vector_T(
		const StateInputVector_T<T>& xu);

	/**
	 * @brief Packs the state portion of a combined state-input vector.
	 *
	 * Reads vx, vy, vz, p, q, r, roll, and pitch from the first eight elements.
	 *
	 * @tparam T Scalar type stored by the vector.
	 * @param[in] xu Combined state-input vector.
	 * @return Structured state.
	 */
	template <typename T> dynamics::State_T<T> pack_state_T(const StateInputVector_T<T>& xu);

	/**
	 * @brief Packs the actuator portion of a combined state-input vector.
	 *
	 * Reads elevator, aileron, rudder, front propulsor, left propulsor, and right propulsor from elements 8 through
	 * 13. Flap and spoiler commands retain their default values.
	 *
	 * @tparam T Scalar type stored by the vector.
	 * @param[in] xu Combined state-input vector.
	 * @return Structured actuator inputs.
	 */
	template <typename T> actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(const StateInputVector_T<T>& xu);

	/**
	 * @brief Packs the virtual wrench portion of a combined virtual state-input vector.
	 *
	 * Reads Fx, Fy, Fz, Mx, My, and Mz from elements 8 through 13.
	 *
	 * @tparam T Scalar type stored by the vector.
	 * @param[in] xu Combined virtual state-input vector.
	 * @return Structured virtual force [N] and moment [N m].
	 */
	template <typename T> dynamics::Wrench_T<T> pack_virtual_inputs_T(const VirtualStateInputVector_T<T>& xu);

	/**
	 * @brief Packs actuator inputs and applies configured fixed controls.
	 *
	 * Reads the six variable actuator inputs from @p xu, then assigns the configured flap and spoiler inputs.
	 *
	 * @tparam T Scalar type stored by the actuator inputs.
	 * @param[in] xu Combined state-input vector.
	 * @param[in] fixed_actuator_inputs Fixed flap and spoiler inputs.
	 * @return Structured variable and fixed actuator inputs.
	 */
	template <typename T>
	actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(const StateInputVector_T<T>& xu,
		const actuators::FixedActuatorInputs& fixed_actuator_inputs);

	/**
	 * @brief Packs a combined state-input vector into an operating point.
	 *
	 * Flap and spoiler inputs retain their default values.
	 *
	 * @tparam T Scalar type stored by the operating point.
	 * @param[in] xu Combined state-input vector.
	 * @return Structured state and actuator inputs.
	 */
	template <typename T> OperatingPoint_T<T> pack_state_input_T(const StateInputVector_T<T>& xu);

	/**
	 * @brief Combines a structured state and actuator inputs into an operating point.
	 *
	 * @tparam T Scalar type stored by the operating point.
	 * @param[in] x Structured state.
	 * @param[in] u Structured actuator inputs.
	 * @return Operating point containing @p x and @p u.
	 */
	template <typename T>
	OperatingPoint_T<T> pack_state_input_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

	/**
	 * @brief Packs a combined virtual state-input vector into a virtual operating point.
	 *
	 * @tparam T Scalar type stored by the virtual operating point.
	 * @param[in] xu Combined virtual state-input vector.
	 * @return Structured state and virtual wrench input.
	 */
	template <typename T> VirtualOperatingPoint_T<T> pack_virtual_state_input_T(const VirtualStateInputVector_T<T>& xu);

} // namespace operating

#include "simulation/operating/public/data/helpers.tpp"
