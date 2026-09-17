#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/control/public/data/types.hpp"
#include "simulation/qp/public/solver.hpp"

#include <array>

namespace allocator
{

	/**
	 * @brief Solves the actuator-increment quadratic program subject to actuator limits.
	 *
	 * The decision vector is the increment from @p u_0. Active actuators use their
	 * configured bounds, while inactive actuators are fixed at @p actuator_target.
	 * If the solver does not report a solution, the failure is logged and @p u_0
	 * is returned.
	 *
	 * @param[in,out] solver Bound-constrained quadratic-program solver.
	 * @param[in] hessian Quadratic objective Hessian.
	 * @param[in] gradient Linear objective gradient.
	 * @param[in] u_0 Actuator input about which increments are computed.
	 * @param[in] actuator_target Required input for inactive actuators.
	 * @param[in] limits Minimum and maximum actuator inputs.
	 * @param[in] actuator_mask Mask identifying actuators available to the allocator.
	 * @return Constrained actuator input, or @p u_0 if the solver fails.
	 */
	control::ControlOutput solve_qp_constrained(qp::Solver& solver,
		const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian,
		const actuators::ActuatorInputsVector_T<double>& gradient,
		const actuators::ActuatorInputsVector_T<double>& u_0,
		const actuators::ActuatorInputsVector_T<double>& actuator_target,
		const actuators::ActuatorLimitsVector& limits,
		const std::array<bool, constants::input_dim>& actuator_mask);

	/**
	 * @brief Solves the actuator-increment quadratic objective without active-actuator bounds.
	 *
	 * Active actuators with nonzero ranges are free in the least-squares solve.
	 * Fixed actuators remain at their configured fixed value, and inactive actuators
	 * are fixed at @p actuator_target.
	 *
	 * @param[in] hessian Quadratic objective Hessian.
	 * @param[in] gradient Linear objective gradient.
	 * @param[in] u_0 Actuator input about which increments are computed.
	 * @param[in] actuator_target Required input for inactive actuators.
	 * @param[in] limits Minimum and maximum actuator inputs used to identify fixed actuators.
	 * @param[in] actuator_mask Mask identifying actuators available to the allocator.
	 * @return Unconstrained actuator input for active, nonfixed actuators.
	 */
	control::ControlOutput solve_qp_unconstrained(
		const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian,
		const actuators::ActuatorInputsVector_T<double>& gradient,
		const actuators::ActuatorInputsVector_T<double>& u_0,
		const actuators::ActuatorInputsVector_T<double>& actuator_target,
		const actuators::ActuatorLimitsVector& limits,
		const std::array<bool, constants::input_dim>& actuator_mask);

} // namespace allocator
