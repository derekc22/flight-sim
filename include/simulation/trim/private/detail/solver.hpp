#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim
{

	/**
	 * @brief Validates trim-solver configuration values.
	 *
	 * @param[in] options Trim-solver options to validate.
	 */
	void validate_trim_solve_options(const TrimSolveOptions& options);

	/**
	 * @brief Solves a bounded nonlinear trim problem.
	 *
	 * @param[in] problem Trim targets, operating conditions, and initial guess.
	 * @param[in] model Automatic-differentiation model and actuator limits.
	 * @param[in] options Solver configuration.
	 * @return Attempted trim solution containing the final candidate and convergence status.
	 */
	TrimSolution
	solve_trim(const TrimProblem& problem, const autodiff::AutoDiffModel& model, TrimSolveOptions options = {});

} // namespace trim
