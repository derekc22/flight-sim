#include <stdexcept>
#include "simulation/qp/public.hpp"

namespace qp {

    static std::size_t validate_dimension(std::size_t dimension) {
        if (dimension == 0) throw std::invalid_argument("qp::Solver dimension must be positive");
        return dimension;
    }

    static void validate_problem(const Problem& problem, std::size_t dimension) {
        const Eigen::Index n = static_cast<Eigen::Index>(dimension);
        if (problem.hessian.rows() != n || problem.hessian.cols() != n) throw std::invalid_argument("qp::Solver::solve hessian dimensions must match solver dimension");
        if (problem.gradient.size() != n) throw std::invalid_argument("qp::Solver::solve gradient dimension must match solver dimension");
        if (problem.lower.size() != n || problem.upper.size() != n) throw std::invalid_argument("qp::Solver::solve bound dimensions must match solver dimension");
        if (!problem.hessian.allFinite() || !problem.gradient.allFinite()) throw std::invalid_argument("qp::Solver::solve objective must be finite");
        if (problem.lower.array().isNaN().any() || problem.upper.array().isNaN().any()) throw std::invalid_argument("qp::Solver::solve bounds cannot contain NaN");
        if ((problem.lower.array() > problem.upper.array()).any()) throw std::invalid_argument("qp::Solver::solve lower bounds cannot exceed upper bounds");
        if (!problem.hessian.isApprox(problem.hessian.transpose())) throw std::invalid_argument("qp::Solver::solve hessian must be symmetric");
    }

    static Status map_status(proxsuite::proxqp::QPSolverOutput status) {
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

    Solver::Solver(std::size_t dimension) : dimension(validate_dimension(dimension)), solver(static_cast<proxsuite::proxqp::dense::isize>(dimension), 0, 0, true) {
        solver.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    }

    Solution Solver::solve(const Problem& problem) {
        validate_problem(problem, dimension);
        const proxsuite::proxqp::dense::Mat<double> hessian = problem.hessian;
        solver.update(hessian, problem.gradient, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, problem.lower, problem.upper, true);
        solver.solve();
        return {
            .x = solver.results.x,
            .status = map_status(solver.results.info.status)
        };
    }

}
