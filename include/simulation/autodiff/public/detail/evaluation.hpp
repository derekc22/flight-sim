#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace autodiff
{

	/**
	 * @brief Computes the net body-frame wrench at an operating point.
	 *
	 * Applies the model's fixed flap and spoiler inputs, computes body gravity from roll and pitch, and combines
	 * aerodynamic, propulsive, and gravitational loads.
	 *
	 * @tparam T Scalar type used for the evaluation.
	 * @param[in] operating_point Reduced state and actuator inputs.
	 * @param[in] model Structural, aerodynamic, propulsion, and actuator model data.
	 * @param[in] conditions Atmospheric, body-wind, and steady-state conditions.
	 * @param[in] dt Time step used by the propulsion evaluation [s].
	 * @return Net body-expressed force [N] and moment [N m].
	 */
	template <typename T>
	dynamics::Wrench_T<T> compute_net_wrench_T(const operating::OperatingPoint_T<T>& operating_point,
		const AutoDiffModel& model,
		const operating::OperatingConditions& conditions,
		T dt);

	/**
	 * @brief Computes the reduced state derivative at an operating point.
	 *
	 * Evaluates the net wrench and then computes translational acceleration, angular acceleration, roll rate, and
	 * pitch rate.
	 *
	 * @tparam T Scalar type used for the evaluation.
	 * @param[in] operating_point Reduced state and actuator inputs.
	 * @param[in] model Structural, aerodynamic, propulsion, and actuator model data.
	 * @param[in] conditions Atmospheric, body-wind, and steady-state conditions.
	 * @param[in] dt Time step used by the propulsion evaluation [s].
	 * @return Reduced state derivative.
	 */
	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_T(const operating::OperatingPoint_T<T>& operating_point,
		const AutoDiffModel& model,
		const operating::OperatingConditions& conditions,
		T dt);

	/**
	 * @brief Computes the reduced state derivative using a virtual wrench input.
	 *
	 * @tparam T Scalar type used for the evaluation.
	 * @param[in] operating_point Reduced state and virtual net-wrench input.
	 * @param[in] model Structural model data.
	 * @return Reduced state derivative.
	 */
	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_T(const operating::VirtualOperatingPoint_T<T>& operating_point,
		const AutoDiffModel& model);

	/**
	 * @brief Computes the reduced state derivative from a supplied net wrench.
	 *
	 * The result is ordered as linear acceleration, angular acceleration, roll rate, and pitch rate. The Euler-rate
	 * calculation is singular when pitch has zero cosine.
	 *
	 * @tparam T Scalar type used for the evaluation.
	 * @param[in] x Reduced state.
	 * @param[in] model Structural model data containing mass and inertia.
	 * @param[in] net_wrench Net body-expressed force [N] and moment [N m].
	 * @return Reduced state derivative.
	 */
	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_from_net_wrench_T(const dynamics::State_T<T>& x,
		const AutoDiffModel& model,
		const dynamics::Wrench_T<T>& net_wrench);

} // namespace autodiff

#include "simulation/autodiff/public/detail/evaluation.tpp"
