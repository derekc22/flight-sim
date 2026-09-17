#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace linearization
{

	/**
	 * @brief Linearizes the reduced aircraft dynamics about an actuator operating point.
	 *
	 * Uses automatic differentiation to compute the continuous-time state Jacobian
	 * A = df/dx and actuator-input Jacobian B = df/du. The output Jacobian remains
	 * the identity and the direct-feedthrough Jacobian remains zero.
	 *
	 * @param[in] model Differentiable structural, aerodynamic, propulsion, and actuator model.
	 * @param[in] operating_point Reduced state and actuator input at which to linearize.
	 * @param[in] conditions Atmospheric, wind, and steady-state evaluation conditions.
	 * @return Continuous local state-space linearization.
	 */
	LocalLinearization linearize_operating_point(const autodiff::AutoDiffModel& model,
		const operating::OperatingPoint_T<double>& operating_point,
		const operating::OperatingConditions& conditions);

	/**
	 * @brief Linearizes the reduced aircraft dynamics about a virtual-wrench operating point.
	 *
	 * Uses automatic differentiation to compute the continuous-time state Jacobian
	 * A_virtual = df/dx and virtual-wrench Jacobian B_virtual = df/dmu. The output
	 * Jacobian remains the identity and the direct-feedthrough Jacobian remains zero.
	 *
	 * @param[in] model Structural model used to evaluate the rigid-body dynamics.
	 * @param[in] virtual_operating_point Reduced state and body-frame net wrench at which to linearize.
	 * @return Continuous local state-space linearization with virtual-wrench inputs.
	 */
	VirtualLocalLinearization linearize_virtual_operating_point(const autodiff::AutoDiffModel& model,
		const operating::VirtualOperatingPoint_T<double>& virtual_operating_point);

} // namespace linearization
