#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility> // For std::pair
#include <stdexcept>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/trim/private.hpp"
#include "simulation/util/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    double send_control_to_solver_space(double u, double limit_min, double limit_max) {
        double mid = 0.5 * (limit_max + limit_min);
        double half_range = 0.5 * (limit_max - limit_min);

        if (half_range <= 0.0) {
            return 0.0;
        }

        double ratio = util::clamp_inside_1((u - mid) / half_range);
        return ratio / util::sqrt(std::max(1.0 - ratio * ratio, constants::eps));
    }

    operating::StateInputVector_T<double> unpack_trim_solver_variables(
        const dynamics::State_T<double>& x, 
        const actuators::ActuatorInputs_T<double>& u, 
        const actuators::ActuatorLimits_T<double>& actuator_limits
    ) {
        operating::StateInputVector_T<double> xu = operating::unpack_state_input_T<double>(x, u);
        const actuators::ActuatorInputsVector_T<double> u_vec = actuators::unpack_actuator_inputs_T<double>(u);
        const actuators::ActuatorLimitsVector_T<double> limits = actuators::unpack_actuator_limits_T<double>(actuator_limits);

        xu(constants::state_dim + 0) = send_control_to_solver_space(u_vec(0), limits(0, 0), limits(0, 1));
        xu(constants::state_dim + 1) = send_control_to_solver_space(u_vec(1), limits(1, 0), limits(1, 1));
        xu(constants::state_dim + 2) = send_control_to_solver_space(u_vec(2), limits(2, 0), limits(2, 1));
        xu(constants::state_dim + 3) = send_control_to_solver_space(u_vec(3), limits(3, 0), limits(3, 1));
        xu(constants::state_dim + 4) = send_control_to_solver_space(u_vec(4), limits(4, 0), limits(4, 1));
        xu(constants::state_dim + 5) = send_control_to_solver_space(u_vec(5), limits(5, 0), limits(5, 1));
        return xu;
    }

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

    dynamics::Wrench compute_trim_wrench(const dynamics::State_T<double>& x, const actuators::ActuatorInputs_T<double>& u, autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions) {
        const dynamics::Twist_T<double> twist = dynamics::build_twist_from_state_T<double>(x);
        const dynamics::Wrench_T<double> net_wrench = autodiff::compute_net_wrench_T<double>(x, twist, u, model, conditions, constants::dt);

        return {
            .F = dynamics::Force{ net_wrench.F },
            .M = dynamics::Moment{ net_wrench.M },
        };
    }

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.operating_point.state = operating::pack_state_T<double>(xu);
        out.operating_point.input = pack_trim_actuator_inputs_T<double>(xu, model.actuator_limits);
        out.conditions = conditions;
        out.wrench = compute_trim_wrench(out.operating_point.state, out.operating_point.input, model, out.conditions);
        out.variables = operating::unpack_state_input_T<double>(out.operating_point.state, out.operating_point.input);
        out.attempted = true;
        out.converged = converged;
        out.iterations = iterations;
        out.residual = TrimResidual_T<double>{
            .vx_dot = residual(0),
            .vy_dot = residual(1),
            .vz_dot = residual(2),
            .p_dot = residual(3),
            .q_dot = residual(4),
            .r_dot = residual(5),
            .phi_dot = residual(6),
            .theta_dot = residual(7),
            .beta_err = residual(8),
            .phi_err = residual(9),
            .theta_err = residual(10),
            .vx_err = residual(11),
            .vz_err = residual(12),
            .psi_dot_err = residual(13)
        };
        out.weighted_residual = TrimResidual_T<double>{
            .vx_dot = weighted_residual(0),
            .vy_dot = weighted_residual(1),
            .vz_dot = weighted_residual(2),
            .p_dot = weighted_residual(3),
            .q_dot = weighted_residual(4),
            .r_dot = weighted_residual(5),
            .phi_dot = weighted_residual(6),
            .theta_dot = weighted_residual(7),
            .beta_err = weighted_residual(8),
            .phi_err = weighted_residual(9),
            .theta_err = weighted_residual(10),
            .vx_err = weighted_residual(11),
            .vz_err = weighted_residual(12),
            .psi_dot_err = weighted_residual(13)
        };
        // out.residual_norm_2 = residual.norm();
        // out.residual_norm_inf = residual_norm_inf(residual);
        out.weighted_residual_norm_2 = weighted_residual.norm();
        out.weighted_residual_norm_inf = residual_norm_inf(weighted_residual);
        return out;
    }

    TrimResidualVector_T<double> compute_trim_residual_vector(
        const operating::StateInputVector_T<double>& xu, 
        autodiff::AutoDiffModel& model,
        const TrimTarget& target, 
        const operating::OperatingConditions& conditions, 
        bool use_physical_controls
    ) {
        return compute_trim_residual_vector_T<double>(xu, model, target, conditions, use_physical_controls);
    }

    TrimResidualJacobian compute_trim_residual_jac(
        const operating::StateInputVector_T<double>& xu, 
        autodiff::AutoDiffModel& model, 
        const TrimTarget& target, 
        const operating::OperatingConditions& conditions, 
        bool use_physical_controls
    ) {
        CppAD::eigen_vector<CppAD::AD<double>> xu_tracked = autodiff::start_autodiff_tracking(xu);
        const operating::StateInputVector_T<CppAD::AD<double>> xu_ad = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::state_input_dim>(xu_tracked);
        const TrimResidualVector_T<CppAD::AD<double>> residual_tracked = compute_trim_residual_vector_T<CppAD::AD<double>>(
            xu_ad,
            model,
            target,
            conditions,
            use_physical_controls
        );
        const CppAD::eigen_vector<CppAD::AD<double>> residual_tracked_cppad = autodiff::cppad_vector_from_eigen_vector(residual_tracked);
        CppAD::ADFun<double> f(xu_tracked, residual_tracked_cppad);
        return autodiff::compute_jac<trim_residual_dim, constants::state_input_dim>(f, xu);
    }

    TrimSolution solve_trim(
        const TrimProblem<double>& problem, 
        autodiff::AutoDiffModel& model, 
        TrimSolveOptions options
    ) {
        validate_trim_solve_options(options);

        const bool use_physical_controls = false;
        operating::StateInputVector_T<double> xu = unpack_trim_solver_variables(problem.state_guess, problem.input_guess, model.actuator_limits);
        TrimResidualVector_T<double> residual = compute_trim_residual_vector(xu, model, problem.target, problem.conditions, use_physical_controls);
        const TrimResidualVector_T<double> weights = fetch_trim_residual_weights(options);
        TrimResidualVector_T<double> weighted_residual = weights.cwiseProduct(residual);
        double damping = options.initial_damping;
        std::size_t iterations_completed = 0;

        for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
            iterations_completed = iteration;
            const double weighted_residual_norm_inf = residual_norm_inf(weighted_residual);

            if (weighted_residual_norm_inf <= options.residual_tolerance) {
                return build_trim_solution(xu, residual, weighted_residual, model, problem.conditions, true, iteration);
            }

            const TrimResidualJacobian jac_raw = compute_trim_residual_jac(xu, model, problem.target, problem.conditions, use_physical_controls);
            const TrimResidualJacobian jac = weights.asDiagonal() * jac_raw;
            const Eigen::Matrix<double, constants::state_input_dim, constants::state_input_dim> hess = jac.transpose() * jac + damping * Eigen::Matrix<double, constants::state_input_dim, constants::state_input_dim>::Identity();
            const operating::StateInputVector_T<double> grad = jac.transpose() * weighted_residual;
            const Eigen::LDLT<Eigen::Matrix<double, constants::state_input_dim, constants::state_input_dim>> ldlt(hess);
            const operating::StateInputVector_T<double> step = ldlt.solve(-grad);

            if (ldlt.info() != Eigen::Success || !step.allFinite()) {
                break;
            }

            if (step.norm() <= options.step_tolerance) {
                return build_trim_solution(xu, residual, weighted_residual, model, problem.conditions, weighted_residual_norm_inf <= options.residual_tolerance, iteration);
            }

            bool accepted = false;
            double step_scale = 1.0;
            const double weighted_residual_norm_2 = weighted_residual.norm();

            while (step_scale >= options.min_step_scale) {
                const operating::StateInputVector_T<double> xu_trial = xu + step_scale * step;
                const TrimResidualVector_T<double> residual_trial = compute_trim_residual_vector(xu_trial, model, problem.target, problem.conditions, use_physical_controls);
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
