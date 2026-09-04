#include "simulation/qp/private/detail/status.hpp"

namespace qp
{

	Status map_status(
		proxsuite::proxqp::QPSolverOutput status)
	{
		switch (status) {
			case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
				return Status::Solved;
			case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
				return Status::MaxIterations;
			case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
				return Status::PrimalInfeasible;
			case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
				return Status::DualInfeasible;
			case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
				return Status::ClosestPrimalFeasible;
			case proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN:
				return Status::NotRun;
		}
		return Status::NotRun;
	}

} // namespace qp
