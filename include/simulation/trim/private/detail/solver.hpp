#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim
{

	/**
	 * @brief Validates trim-solver configuration values.
	 *
	 * Residual and step tolerances and initial damping must be nonnegative. Damping growth must exceed one; residual
	 * scales must be positive; backtracking scale must lie in `(0, 1)`; and minimum step scale must lie in `(0, 1]`.
	 *
	 * @param[in] options Trim-solver options to validate.
	 */
	void validate_trim_solve_options(const TrimSolveOptions& options);

	/**
	 * @brief Solves a bounded nonlinear trim problem.
	 *
	 * Uses weighted residuals, automatic-differentiation Jacobians, damped quadratic subproblems, actuator bounds,
	 * and backtracking. A nonconverged solve is returned as a trim solution rather than thrown.
	 *
	 * @param[in] problem Trim targets, operating conditions, and initial guess.
	 * @param[in] model Automatic-differentiation model and actuator limits.
	 * @param[in] options Solver configuration.
	 * @return Attempted trim solution containing the final candidate and convergence status.
	 */
	TrimSolution
	solve_trim(const TrimProblem& problem, const autodiff::AutoDiffModel& model, TrimSolveOptions options = {});

} // namespace trim
