#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"

namespace trim
{

	/**
	 * @brief Computes the trim-residual vector for a state-input candidate.
	 *
	 * @tparam T Scalar type used for the residual evaluation.
	 * @param[in] xu Candidate state-input vector.
	 * @param[in] model Automatic-differentiation model used to evaluate state derivatives.
	 * @param[in] target Desired trim quantities.
	 * @param[in] conditions Atmospheric, wind, and steady-state conditions.
	 * @return Trim-residual vector in the documented residual order.
	 */
	template <typename T>
	TrimResidualVector_T<T> compute_trim_residual_vector_T(const operating::StateInputVector_T<T>& xu,
		const autodiff::AutoDiffModel& model,
		const TrimTarget& target,
		const operating::OperatingConditions& conditions);

	/**
	 * @brief Computes the infinity norm of a trim-residual vector.
	 *
	 * @param[in] residual Trim-residual vector.
	 * @return Largest absolute residual component.
	 */
	double residual_norm_inf(const TrimResidualVector_T<double>& residual);

	/**
	 * @brief Computes the trim-residual Jacobian using automatic differentiation.
	 *
	 * @param[in] xu State-input vector at which the Jacobian is evaluated.
	 * @param[in] model Automatic-differentiation model used for residual evaluation.
	 * @param[in] target Desired trim quantities.
	 * @param[in] conditions Atmospheric, wind, and steady-state conditions.
	 * @return Jacobian of trim residuals with respect to state and actuator inputs.
	 */
	TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu,
		const autodiff::AutoDiffModel& model,
		const TrimTarget& target,
		const operating::OperatingConditions& conditions);

} // namespace trim

#include "simulation/trim/private/detail/residual.tpp"
