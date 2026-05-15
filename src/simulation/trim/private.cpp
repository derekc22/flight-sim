#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility> // For std::pair
#include <stdexcept>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/trim/private.hpp"
#include "simulation/util/cppad/public.hpp"
#include "simulation/util/public.hpp"

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

    TrimVariablesVector_T<double> unpack_trim_solver_variables(const dynamics::State_T<double>& x, const actuators::ActuatorInputs_T<double>& u, const actuators::ActuatorLimits_T<double>& actuator_limits) {
        TrimVariablesVector_T<double> z = unpack_trim_variables_T<double>(x, u);
        const actuators::ActuatorInputsVector_T<double> u_vec = actuators::unpack_actuator_inputs_T<double>(u);
        const actuators::ActuatorLimitsVector_T<double> limits = actuators::unpack_actuator_limits_T<double>(actuator_limits);

        z(constants::state_dim + 0) = send_control_to_solver_space(u_vec(0), limits(0, 0), limits(0, 1));
        z(constants::state_dim + 1) = send_control_to_solver_space(u_vec(1), limits(1, 0), limits(1, 1));
        z(constants::state_dim + 2) = send_control_to_solver_space(u_vec(2), limits(2, 0), limits(2, 1));
        z(constants::state_dim + 3) = send_control_to_solver_space(u_vec(3), limits(3, 0), limits(3, 1));
        z(constants::state_dim + 4) = send_control_to_solver_space(u_vec(4), limits(4, 0), limits(4, 1));
        z(constants::state_dim + 5) = send_control_to_solver_space(u_vec(5), limits(5, 0), limits(5, 1));
        return z;
    }

    TrimResidualVector_T<double> trim_residual_weights(const TrimSolveOptions& options) {
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

    double _residual_norm_inf(const TrimResidualVector_T<double>& residual) {
        return residual.cwiseAbs().maxCoeff();
    }

    void _validate_trim_solve_options(const TrimSolveOptions& options) {
        if (options.residual_tolerance < 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: residual_tolerance must be non-negative");
        if (options.step_tolerance < 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: step_tolerance must be non-negative");
        if (options.initial_damping < 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: initial_damping must be non-negative");
        if (options.damping_growth <= 1.0) throw std::invalid_argument("trim::_validate_trim_solve_options: damping_growth must be greater than 1");
        if (options.linear_accel_scale <= 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: linear_accel_scale must be positive");
        if (options.angular_accel_scale <= 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: angular_accel_scale must be positive");
        if (options.angle_rate_scale <= 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: angle_rate_scale must be positive");
        if (options.angle_err_scale <= 0.0) throw std::invalid_argument("trim::_validate_trim_solve_options: angle_err_scale must be positive");
        if (options.backtrack_scale <= 0.0 || options.backtrack_scale >= 1.0) throw std::invalid_argument("trim::_validate_trim_solve_options: backtrack_scale must be in (0, 1)");
        if (options.min_step_scale <= 0.0 || options.min_step_scale > 1.0) throw std::invalid_argument("trim::_validate_trim_solve_options: min_step_scale must be in (0, 1]");
    }

    dynamics::Wrench compute_trim_wrench(const dynamics::State_T<double>& x, const actuators::ActuatorInputs_T<double>& u, const TrimModel& model, const TrimConditions& conditions) {
        const dynamics::Twist_T<double> twist = build_twist_from_trim_state_T<double>(x);
        const dynamics::Wrench_T<double> net_wrench = compute_trim_net_wrench_T<double>(x, twist, u, model, conditions);

        return {
            .F = dynamics::Force{ net_wrench.F },
            .M = dynamics::Moment{ net_wrench.M },
        };
    }

    TrimSolution build_trim_solution(const TrimVariablesVector_T<double>& z, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, const TrimModel& model, const TrimConditions& conditions, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.state = pack_trim_state_T<double>(z);
        out.input = pack_trim_actuator_inputs_T<double>(z, model.actuator_limits);
        out.conditions = conditions;
        out.wrench = compute_trim_wrench(out.state, out.input, model, out.conditions);
        out.variables = unpack_trim_variables_T<double>(out.state, out.input);
        out.attempted = true;
        out.converged = converged;
        out.iterations = iterations;
        out.residual = TrimResidual<double>{
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
        out.weighted_residual = TrimResidual<double>{
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
        // out.residual_norm_inf = _residual_norm_inf(residual);
        out.weighted_residual_norm_2 = weighted_residual.norm();
        out.weighted_residual_norm_inf = _residual_norm_inf(weighted_residual);
        return out;
    }

    TrimResidualVector_T<double> compute_trim_residual_vector(const TrimVariablesVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        return compute_trim_residual_vector_T<double>(z, model, target, conditions, use_physical_controls);
    }

    TrimResidualJacobian compute_trim_residual_jac(const TrimVariablesVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        CppAD::eigen_vector<CppAD::AD<double>> z_tracked_cppad = util::start_autodiff_tracking(z);
        const TrimResidualVector_T<CppAD::AD<double>> residual_tracked = compute_trim_residual_vector_T<CppAD::AD<double>>(
            util::eigen_vector_from_cppad_vector<CppAD::AD<double>, trim_variable_dim>(z_tracked_cppad),
            model,
            target,
            conditions,
            use_physical_controls
        );
        const CppAD::eigen_vector<CppAD::AD<double>> residual_tracked_cppad = util::cppad_vector_from_eigen_vector(residual_tracked);
        CppAD::ADFun<double> f(z_tracked_cppad, residual_tracked_cppad);
        return util::compute_jac<trim_residual_dim, trim_variable_dim>(f, z);
    }

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModel& model, TrimSolveOptions options) {
        _validate_trim_solve_options(options);

        const bool use_physical_controls = false;
        TrimVariablesVector_T<double> z = unpack_trim_solver_variables(problem.state_guess, problem.input_guess, model.actuator_limits);
        TrimResidualVector_T<double> residual = compute_trim_residual_vector(z, model, problem.target, problem.conditions, use_physical_controls);
        const TrimResidualVector_T<double> weights = trim_residual_weights(options);
        TrimResidualVector_T<double> weighted_residual = weights.cwiseProduct(residual);
        double damping = options.initial_damping;
        std::size_t iterations_completed = 0;

        for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
            iterations_completed = iteration;
            const double weighted_residual_norm_inf = _residual_norm_inf(weighted_residual);

            if (weighted_residual_norm_inf <= options.residual_tolerance) {
                return build_trim_solution(z, residual, weighted_residual, model, problem.conditions, true, iteration);
            }

            const TrimResidualJacobian jac_raw = compute_trim_residual_jac(z, model, problem.target, problem.conditions, use_physical_controls);
            const TrimResidualJacobian jac = weights.asDiagonal() * jac_raw;
            const Eigen::Matrix<double, trim_variable_dim, trim_variable_dim> hess = jac.transpose() * jac + damping * Eigen::Matrix<double, trim_variable_dim, trim_variable_dim>::Identity();
            const TrimVariablesVector_T<double> grad = jac.transpose() * weighted_residual;
            const Eigen::LDLT<Eigen::Matrix<double, trim_variable_dim, trim_variable_dim>> ldlt(hess);
            const TrimVariablesVector_T<double> step = ldlt.solve(-grad);

            if (ldlt.info() != Eigen::Success || !step.allFinite()) {
                break;
            }

            if (step.norm() <= options.step_tolerance) {
                return build_trim_solution(z, residual, weighted_residual, model, problem.conditions, weighted_residual_norm_inf <= options.residual_tolerance, iteration);
            }

            bool accepted = false;
            double step_scale = 1.0;
            const double weighted_residual_norm_2 = weighted_residual.norm();

            while (step_scale >= options.min_step_scale) {
                const TrimVariablesVector_T<double> z_trial = z + step_scale * step;
                const TrimResidualVector_T<double> residual_trial = compute_trim_residual_vector(z_trial, model, problem.target, problem.conditions, use_physical_controls);
                const double weighted_residual_trial_norm_2 = weights.cwiseProduct(residual_trial).norm();

                if (weighted_residual_trial_norm_2 < weighted_residual_norm_2) {
                    z = z_trial;
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

        return build_trim_solution(z, residual, weighted_residual, model, problem.conditions, false, iterations_completed);
    }

    /** @deprecated */
    // void update_actuators_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol) {
    //     surface_actuators.aileron.prev_cmd = trim_sol.input.aileron_cmd;
    //     surface_actuators.elevator.prev_cmd = trim_sol.input.elevator_cmd;
    //     surface_actuators.rudder.prev_cmd = trim_sol.input.rudder_cmd;

    //     propulsor_actuators.front_propulsor.prev_cmd = trim_sol.input.front_propulsor_cmd;
    //     propulsor_actuators.left_propulsor.prev_cmd = trim_sol.input.left_propulsor_cmd;
    //     propulsor_actuators.right_propulsor.prev_cmd = trim_sol.input.right_propulsor_cmd;
    // }

}
