#include <algorithm>
#include <limits>
#include <stdexcept>
#include <spdlog/spdlog.h>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/qp/public/solver.hpp"
#include "simulation/trim/private/data/helpers.hpp"
#include "simulation/trim/private/detail/residual.hpp"
#include "simulation/trim/private/detail/solver.hpp"

namespace trim {

    void validate_trim_solve_options(const TrimSolveOptions& options) {
        if (options.residual_tolerance < 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: residual_tolerance must be non-negative");
        if (options.step_tolerance < 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: step_tolerance must be non-negative");
        if (options.initial_damping < 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: initial_damping must be non-negative");
        if (options.damping_growth <= 1.0) throw std::invalid_argument("trim::validate_trim_solve_options: damping_growth must be greater than 1");
        if (options.linear_accel_scale <= 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: linear_accel_scale must be positive");
        if (options.angular_accel_scale <= 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: angular_accel_scale must be positive");
        if (options.angle_rate_scale <= 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: angle_rate_scale must be positive");
        if (options.angle_err_scale <= 0.0) throw std::invalid_argument("trim::validate_trim_solve_options: angle_err_scale must be positive");
        if (options.backtrack_scale <= 0.0 || options.backtrack_scale >= 1.0) throw std::invalid_argument("trim::validate_trim_solve_options: backtrack_scale must be in (0, 1)");
        if (options.min_step_scale <= 0.0 || options.min_step_scale > 1.0) throw std::invalid_argument("trim::validate_trim_solve_options: min_step_scale must be in (0, 1]");
    }

    TrimSolution solve_trim(const TrimProblem& problem, const autodiff::AutoDiffModel& model, TrimSolveOptions options) {
        validate_trim_solve_options(options);

        operating::StateInputVector_T<double> xu = operating::unpack_state_input_T<double>(problem.initial_guess.state, problem.initial_guess.input);

        const actuators::ActuatorLimitsVector actuator_limits = actuators::unpack_actuator_limits(model.actuator_limits);
        xu.tail<constants::input_dim>() = xu.tail<constants::input_dim>().cwiseMax(actuator_limits.col(0)).cwiseMin(actuator_limits.col(1));

        TrimResidualVector_T<double> residual = compute_trim_residual_vector_T<double>(xu, model, problem.target, problem.conditions);

        const TrimResidualVector_T<double> weights = fetch_trim_residual_weights(options);
        TrimResidualVector_T<double> weighted_residual = weights.cwiseProduct(residual);

        double damping = options.initial_damping;
        std::size_t iterations_completed = 0;

        qp::Solver solver(constants::state_input_dim);

        for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
            iterations_completed = iteration;
            const double weighted_residual_norm_inf = residual_norm_inf(weighted_residual);

            if (weighted_residual_norm_inf <= options.residual_tolerance) {
                return build_trim_solution(xu, residual, weighted_residual, model, problem.conditions, true, iteration);
            }

            const TrimResidualJacobian jac_raw = compute_trim_residual_jac(xu, model, problem.target, problem.conditions);
            const TrimResidualJacobian jac = weights.asDiagonal() * jac_raw;

            const constants::MatrixX_T<double, constants::state_input_dim, constants::state_input_dim> hess = jac.transpose() * jac + damping * constants::I_T<double, constants::state_input_dim>;

            const operating::StateInputVector_T<double> grad = jac.transpose() * weighted_residual;

            Eigen::VectorXd lower = Eigen::VectorXd::Constant(constants::state_input_dim, -std::numeric_limits<double>::infinity());
            Eigen::VectorXd upper = Eigen::VectorXd::Constant(constants::state_input_dim, std::numeric_limits<double>::infinity());
            lower.tail<constants::input_dim>() = actuator_limits.col(0) - xu.tail<constants::input_dim>();
            upper.tail<constants::input_dim>() = actuator_limits.col(1) - xu.tail<constants::input_dim>();

            const qp::Problem step_problem{
                .hessian = hess,
                .gradient = grad,
                .lower = lower,
                .upper = upper
            };

            const qp::Solution step_solution = solver.solve(step_problem);

            if (step_solution.status != qp::Status::Solved || !step_solution.x.allFinite()) {
                spdlog::error("trim::solve_trim QP solve failed with status {}", static_cast<int>(step_solution.status));
                break;
            }

            const operating::StateInputVector_T<double> step = step_solution.x;

            if (step.norm() <= options.step_tolerance) {
                return build_trim_solution(
                    xu, 
                    residual,
                    weighted_residual,
                    model,
                    problem.conditions,
                    weighted_residual_norm_inf <= options.residual_tolerance,
                    iteration
                );
            }

            bool accepted = false;
            double step_scale = 1.0;
            const double weighted_residual_norm_2 = weighted_residual.norm();

            while (step_scale >= options.min_step_scale) {
                const operating::StateInputVector_T<double> xu_trial = xu + step_scale * step;
                const TrimResidualVector_T<double> residual_trial = compute_trim_residual_vector_T<double>(xu_trial, model, problem.target, problem.conditions);
                const double weighted_residual_trial_norm_2 = weights.cwiseProduct(residual_trial).norm();

                if (weighted_residual_trial_norm_2 < weighted_residual_norm_2) {
                    xu = xu_trial;
                    residual = residual_trial;
                    weighted_residual = weights.cwiseProduct(residual_trial);
                    damping = std::max(options.initial_damping, damping / options.damping_growth);
                    accepted = true;
                    break;
                }
                step_scale *= options.backtrack_scale;
            }

            if (!accepted) {
                damping *= options.damping_growth;
            }
        }

        return build_trim_solution(xu, residual, weighted_residual, model, problem.conditions, false, iterations_completed);
    }

}
