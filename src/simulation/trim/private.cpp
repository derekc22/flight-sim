#include <algorithm>
#include <limits>
#include <stdexcept>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/trim/private.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/qp/public.hpp"

namespace trim {

    TrimResidualVector_T<double> fetch_trim_residual_weights(const TrimSolveOptions& options) {
        TrimResidualVector_T<double> w;
        w << 1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.vel_err_scale,
             1.0 / options.vel_err_scale,
             1.0 / options.angle_rate_scale;
        return w;
    }

    double residual_norm_inf(const TrimResidualVector_T<double>& residual) {
        return residual.cwiseAbs().maxCoeff();
    }

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

    dynamics::Wrench compute_trim_wrench(const operating::OperatingPoint& operating_point, autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions) {
        const dynamics::Wrench_T<double> net_wrench = autodiff::compute_net_wrench_T<double>(operating_point.state, operating_point.input, model, conditions, constants::dt);

        return {
            .F = dynamics::Force{ net_wrench.F },
            .M = dynamics::Moment{ net_wrench.M },
        };
    }

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.operating_point.state = operating::pack_state_T<double>(xu);
        out.operating_point.input = pack_trim_actuator_inputs_T<double>(xu, model.fixed_actuator_inputs);
        out.conditions = conditions;
        out.wrench = compute_trim_wrench(out.operating_point, model, out.conditions);
        out.variables = operating::unpack_state_input_T<double>(out.operating_point.state, out.operating_point.input);
        out.attempted = true;
        out.converged = converged;
        out.iterations = iterations;
        out.residual = pack_trim_residual_T(residual);
        out.weighted_residual = pack_trim_residual_T(weighted_residual);
        // out.residual_norm_2 = residual.norm();
        // out.residual_norm_inf = residual_norm_inf(residual);
        out.weighted_residual_norm_2 = weighted_residual.norm();
        out.weighted_residual_norm_inf = residual_norm_inf(weighted_residual);
        return out;
    }

    TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu, autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions) {

        CppAD::eigen_vector<CppAD::AD<double>> xu_tracked = autodiff::start_autodiff_tracking(xu);  // start of autodiff tracking

        const operating::StateInputVector_T<CppAD::AD<double>> xu_eigen = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::state_input_dim>(xu_tracked);

        const TrimResidualVector_T<CppAD::AD<double>> residual_tracked = compute_trim_residual_vector_T<CppAD::AD<double>>(xu_eigen, model, target, conditions);
        const CppAD::eigen_vector<CppAD::AD<double>> residual_tracked_cppad = autodiff::cppad_vector_from_eigen_vector(residual_tracked);

        CppAD::ADFun<double> f(xu_tracked, residual_tracked_cppad); // end of autodiff tracking

        return autodiff::compute_jac<trim_residual_dim, constants::state_input_dim>(f, xu);
    }

    TrimSolution solve_trim(const TrimProblem& problem, autodiff::AutoDiffModel& model, TrimSolveOptions options) {
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

            const constants::MatrixX_T<double, constants::state_input_dim, constants::state_input_dim> hess = jac.transpose() * jac + damping * constants::IX_T<double, constants::state_input_dim>;

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
