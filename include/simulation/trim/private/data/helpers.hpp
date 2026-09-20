#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim
{

	/**
	 * @brief Packs a trim-residual vector into named fields.
	 *
	 * @tparam T Scalar type stored by the residual.
	 * @param[in] residual Trim-residual vector.
	 * @return Trim residual with named fields.
	 */
	template <typename T> TrimResidual_T<T> pack_trim_residual_T(const TrimResidualVector_T<T>& residual);

	/**
	 * @brief Builds reciprocal scaling weights for the trim residual.
	 *
	 * @param[in] options Solver options containing residual scales.
	 * @return Residual weights in trim-residual vector order.
	 */
	constants::MatrixX_T<double, constants::nr, 1> fetch_trim_residual_weights(const TrimSolveOptions& options);

	/**
	 * @brief Builds a complete trim solution from the current solver state.
	 *
	 * @param[in] xu Current combined state-input vector.
	 * @param[in] residual Raw trim-residual vector.
	 * @param[in] weighted_residual Weighted trim-residual vector.
	 * @param[in] model Automatic-differentiation model used to evaluate the net wrench.
	 * @param[in] conditions Operating conditions for the solution.
	 * @param[in] converged Whether the solver satisfied its convergence criteria.
	 * @param[in] iterations Number of completed solver iterations.
	 * @return Populated trim solution marked as attempted.
	 */
	TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu,
		const TrimResidualVector_T<double>& residual,
		const TrimResidualVector_T<double>& weighted_residual,
		const autodiff::AutoDiffModel& model,
		const operating::OperatingConditions& conditions,
		bool converged,
		std::size_t iterations);

} // namespace trim

#include "simulation/trim/private/data/helpers.tpp"
