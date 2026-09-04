#include "simulation/qp/public/solver.hpp"

#include "simulation/qp/private/detail/status.hpp"
#include "simulation/qp/private/detail/validation.hpp"

namespace qp
{

	Solver::Solver(
		std::size_t dimension)
		: dimension(validate_dimension(dimension)),
		  solver(static_cast<proxsuite::proxqp::dense::isize>(dimension), 0, 0, true)
	{
		solver.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
	}

	Solution Solver::solve(
		const Problem& problem)
	{
		validate_problem(problem, dimension);
		const proxsuite::proxqp::dense::Mat<double> hessian = problem.hessian;
		solver.update(hessian,
			problem.gradient,
			proxsuite::nullopt,
			proxsuite::nullopt,
			proxsuite::nullopt,
			proxsuite::nullopt,
			proxsuite::nullopt,
			problem.lower,
			problem.upper,
			true);
		solver.solve();
		return {.x = solver.results.x, .status = map_status(solver.results.info.status)};
	}

} // namespace qp
