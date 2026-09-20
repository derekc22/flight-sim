#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"

namespace actuators
{

	/**
	 * @brief Packs an actuator-input vector into grouped surface and propulsor inputs.
	 *
	 * @tparam T Scalar type stored by the actuator inputs.
	 * @param[in] u Actuator-input vector in actuator command units.
	 * @return Grouped surface and propulsor actuator inputs.
	 */
	template <typename T> ActuatorInputs_T<T> pack_actuator_inputs_T(const ActuatorInputsVector_T<T>& u);

	/**
	 * @brief Unpacks grouped actuator inputs into an actuator-input vector.
	 *
	 * @tparam T Scalar type stored by the actuator inputs.
	 * @param[in] u Grouped surface and propulsor actuator inputs.
	 * @return Actuator-input vector in actuator command units.
	 */
	template <typename T> ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u);

	/**
	 * @brief Packs an actuator-limit matrix into grouped minimum and maximum limits.
	 *
	 * @param[in] limits Actuator-limit matrix in actuator command units.
	 * @return Grouped minimum and maximum actuator limits.
	 */
	ActuatorLimits pack_actuator_limits(const ActuatorLimitsVector& limits);

	/**
	 * @brief Unpacks grouped actuator limits into an actuator-limit matrix.
	 *
	 * @param[in] limits Grouped minimum and maximum actuator limits.
	 * @return Actuator-limit matrix in actuator command units.
	 */
	ActuatorLimitsVector unpack_actuator_limits(const ActuatorLimits& limits);

	/**
	 * @brief Combines surface and propulsor inputs into grouped actuator inputs.
	 *
	 * @tparam T Scalar type stored by the actuator inputs.
	 * @param[in] u_surface Surface-actuator inputs.
	 * @param[in] u_propulsor Propulsor-actuator inputs.
	 * @return Grouped surface and propulsor actuator inputs.
	 */
	template <typename T>
	ActuatorInputs_T<T> pack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface,
		const PropulsorActuatorInputs_T<T>& u_propulsor);

	/**
	 * @brief Unpacks separate surface and propulsor inputs into an actuator-input vector.
	 *
	 * @tparam T Scalar type stored by the actuator inputs.
	 * @param[in] u_surface Surface-actuator inputs.
	 * @param[in] u_propulsor Propulsor-actuator inputs.
	 * @return Actuator-input vector in actuator command units.
	 */
	template <typename T>
	ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface,
		const PropulsorActuatorInputs_T<T>& u_propulsor);

	/**
	 * @brief Collects minimum and maximum limits from surface and propulsor actuators.
	 *
	 * @param[in] surface_actuators Surface actuators containing their configured limits.
	 * @param[in] propulsor_actuators Propulsor actuators containing their configured limits.
	 * @return Grouped minimum and maximum actuator limits.
	 */
	ActuatorLimits pack_actuator_limits(const SurfaceActuators& surface_actuators,
		const PropulsorActuators& propulsor_actuators);

	/**
	 * @brief Collects actuator limits into an actuator-limit matrix.
	 *
	 * @param[in] surface_actuators Surface actuators containing their configured limits.
	 * @param[in] propulsor_actuators Propulsor actuators containing their configured limits.
	 * @return Actuator-limit matrix in actuator command units.
	 */
	ActuatorLimitsVector unpack_actuator_limits(const SurfaceActuators& surface_actuators,
		const PropulsorActuators& propulsor_actuators);

	/**
	 * @brief Computes neutral actuator inputs from the configured limits.
	 *
	 * @param[in] surface_actuators Surface actuators containing their configured limits.
	 * @param[in] propulsor_actuators Propulsor actuators containing their configured limits.
	 * @return Grouped neutral actuator inputs.
	 */
	ActuatorInputs_T<double> get_neutral_actuator_inputs(const SurfaceActuators& surface_actuators,
		const PropulsorActuators& propulsor_actuators);

	/**
	 * @brief Unpacks surface-actuator inputs into a vector.
	 *
	 * @param[in] u_surface Surface-actuator inputs.
	 * @return Surface-actuator input vector in actuator command units.
	 */
	SurfaceActuatorInputsVector unpack_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface);

	/**
	 * @brief Unpacks propulsor-actuator inputs into a vector.
	 *
	 * @param[in] u_propulsor Propulsor-actuator inputs.
	 * @return Propulsor-actuator input vector in actuator command units.
	 */
	PropulsorActuatorInputsVector unpack_propulsor_actuator_inputs(
		const PropulsorActuatorInputs_T<double>& u_propulsor);

	/**
	 * @brief Returns the command for a selected surface actuator.
	 *
	 * @tparam T Scalar type stored by the surface-actuator inputs.
	 * @param[in] actuator_id Surface actuator to select.
	 * @param[in] u Surface-actuator inputs.
	 * @return Selected surface-actuator command in its command units.
	 */
	template <typename T>
	T get_surface_actuator_input_T(SurfaceActuatorID actuator_id, const SurfaceActuatorInputs_T<T>& u);

} // namespace actuators

#include "simulation/actuators/public/data/helpers.tpp"
