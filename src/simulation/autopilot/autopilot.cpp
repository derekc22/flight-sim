#include "simulation/autopilot/autopilot.hpp"
#include <algorithm>
#include <sstream>
#include <utility> // For std::pair
#include <stdexcept>

namespace autopilot { // to encompass autonomy and trim

    static double _send_control_to_solver_space(double u, double limit) {
        if (limit <= 0.0) {
            return 0.0;
        }

        const double ratio = global::clamp_inside_1(u / limit);
        return ratio / std::sqrt(std::max(1.0 - ratio * ratio, global::eps));
    }

    static TrimVariableVector_T<double> _pack_trim_solver_variables(const TrimState<double>& x, const TrimInput<double>& u, const control::ControlSurfaceLimits& limits) {
        TrimVariableVector_T<double> z = pack_trim_variables_T<double>(x, u);
        z(8) = _send_control_to_solver_space(u.elevator, limits.elevator_max);
        z(9) = _send_control_to_solver_space(u.aileron, limits.aileron_max);
        z(10) = _send_control_to_solver_space(u.rudder, limits.rudder_max);
        return z;
    }

    static TrimResidualVector_T<double> _trim_residual_weights(const TrimSolveOptions& options) {
        TrimResidualVector_T<double> w;
        w << 1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_error_scale,
             1.0 / options.angle_error_scale,
             1.0 / options.angle_error_scale;
        return w;
    }

    static double _residual_norm_inf(const TrimResidualVector_T<double>& residual) {
        return residual.cwiseAbs().maxCoeff();
    }

    static void _validate_trim_solve_options(const TrimSolveOptions& options) {
        if (options.residual_tolerance < 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: residual_tolerance must be nonnegative");
        if (options.step_tolerance < 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: step_tolerance must be nonnegative");
        if (options.initial_damping < 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: initial_damping must be nonnegative");
        if (options.damping_growth <= 1.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: damping_growth must be greater than 1");
        if (options.linear_accel_scale <= 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: linear_accel_scale must be positive");
        if (options.angular_accel_scale <= 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: angular_accel_scale must be positive");
        if (options.angle_rate_scale <= 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: angle_rate_scale must be positive");
        if (options.angle_error_scale <= 0.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: angle_error_scale must be positive");
        if (options.backtrack_scale <= 0.0 || options.backtrack_scale >= 1.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: backtrack_scale must be in (0, 1)");
        if (options.min_step_scale <= 0.0 || options.min_step_scale > 1.0) throw std::invalid_argument("autopilot::_validate_trim_solve_options: min_step_scale must be in (0, 1]");
    }

    static TrimSolution _build_trim_solution(const TrimVariableVector_T<double>& z, const TrimResidualVector_T<double>& residual, const TrimModel& model, const TrimConditions& conditions, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.state = unpack_trim_state_T<double>(z);
        out.input = _unpack_trim_solver_input_T<double>(z, model.control.limits);
        out.conditions = conditions;
        out.variables = pack_trim_variables_T<double>(out.state, out.input);
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
            .beta_error = residual(8),
            .phi_error = residual(9),
            .theta_error = residual(10),
        };
        out.residual_norm_2 = residual.norm();
        out.residual_norm_inf = _residual_norm_inf(residual);
        return out;
    }

    TrimResidualVector_T<double> compute_trim_residual_vector(const TrimVariableVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        return compute_trim_residual_vector_T<double>(z, model, target, conditions, use_physical_controls);
    }

    TrimResidualJacobian compute_trim_residual_jac(const TrimVariableVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        CppAD::eigen_vector<CppAD::AD<double>> z_tracked_cppad = global::start_autodiff_tracking(z);
        const TrimResidualVector_T<CppAD::AD<double>> y_tracked = compute_trim_residual_vector_T<CppAD::AD<double>>(
            global::eigen_vector_from_cppad_vector<CppAD::AD<double>, trim_variable_dofs>(z_tracked_cppad),
            model,
            target,
            conditions,
            use_physical_controls
        );
        const CppAD::eigen_vector<CppAD::AD<double>> y_tracked_cppad = global::cppad_vector_from_eigen_vector(y_tracked);
        CppAD::ADFun<double> f(z_tracked_cppad, y_tracked_cppad);
        return global::compute_jac<trim_residual_dofs, trim_variable_dofs>(f, z);
    }

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModel& model, TrimSolveOptions options) {
        _validate_trim_solve_options(options);

        const bool use_physical_controls = false;
        TrimVariableVector_T<double> z = _pack_trim_solver_variables(problem.state_guess, problem.input_guess, model.control.limits);
        TrimResidualVector_T<double> residual = compute_trim_residual_vector(z, model, problem.target, problem.conditions, use_physical_controls);
        double damping = options.initial_damping;
        std::size_t iterations_completed = 0;
        const TrimResidualVector_T<double> weights = _trim_residual_weights(options);

        for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
            iterations_completed = iteration;
            const TrimResidualVector_T<double> weighted_residual = weights.cwiseProduct(residual);
            const double residual_norm_inf = _residual_norm_inf(residual);

            if (residual_norm_inf <= options.residual_tolerance) {
                return _build_trim_solution(z, residual, model, problem.conditions, true, iteration);
            }

            const TrimResidualJacobian jac_raw = compute_trim_residual_jac(z, model, problem.target, problem.conditions, use_physical_controls);
            const TrimResidualJacobian jac = weights.asDiagonal() * jac_raw;
            const Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs> hess = jac.transpose() * jac + damping * Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs>::Identity();
            const TrimVariableVector_T<double> grad = jac.transpose() * weighted_residual;
            const Eigen::LDLT<Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs>> ldlt(hess);
            const TrimVariableVector_T<double> step = ldlt.solve(-grad);

            if (ldlt.info() != Eigen::Success || !step.allFinite()) {
                break;
            }

            if (step.norm() <= options.step_tolerance) {
                return _build_trim_solution(z, residual, model, problem.conditions, residual_norm_inf <= options.residual_tolerance, iteration);
            }

            bool accepted = false;
            double step_scale = 1.0;
            const double residual_norm_2 = weighted_residual.norm();

            while (step_scale >= options.min_step_scale) {
                const TrimVariableVector_T<double> z_trial = z + step_scale * step;
                const TrimResidualVector_T<double> residual_trial = compute_trim_residual_vector(z_trial, model, problem.target, problem.conditions, use_physical_controls);
                const double residual_trial_norm_2 = weights.cwiseProduct(residual_trial).norm();

                if (residual_trial_norm_2 < residual_norm_2) {
                    z = z_trial;
                    residual = residual_trial;
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

        return _build_trim_solution(z, residual, model, problem.conditions, false, iterations_completed);
    }

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind) {
        const TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .control = aircraft.control_properties,
            .fixed_controls = TrimFixedControls{
                .flap = 0.0,
                .spoiler = 0.0,
            },
        };

        const TrimProblem<double> problem{
            .target = TrimTarget{
                .beta = aircraft.aerodynamicState(aircraft.FRDFrameNED, wind).beta.data,
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
            },
            .conditions = TrimConditions{
                .rho = aircraft.atmosphericState(aircraft.FRDFrameECEF).rho,
                .windB = wind,
            },
            .state_guess = TrimState<double>{
                .vx = aircraft.FRDFrameNED.vB_BN.data.x(),
                .vy = aircraft.FRDFrameNED.vB_BN.data.y(),
                .vz = aircraft.FRDFrameNED.vB_BN.data.z(),
                .p = aircraft.FRDFrameNED.wB_BN.p(),
                .q = aircraft.FRDFrameNED.wB_BN.q(),
                .r = aircraft.FRDFrameNED.wB_BN.r(),
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
            },
            .input_guess = TrimInput<double>{},
        };

        const TrimSolution trim_sol = solve_trim(problem, model);
        if (!trim_sol.converged) { std::cerr << "autopilot::inspect_trim: Warning, trim failed to converge" << std::endl; }

        return trim_sol;
    }


    std::string print_trim_solution(const TrimSolution& trim_sol) {
        dynamics::Twist_T<double> trim_sol_twist;
        trim_sol_twist.v << trim_sol.state.vx, trim_sol.state.vy, trim_sol.state.vz;
        trim_sol_twist.w << trim_sol.state.p, trim_sol.state.q, trim_sol.state.r;
        const aerodynamics::AerodynamicState_T<double> trim_sol_ads = aerodynamics::compute_aerodynamic_state_T<double>(trim_sol_twist, trim_sol.conditions.windB);
        const dynamics::AngularVelocity trim_w{ Eigen::Vector3d(trim_sol.state.p, trim_sol.state.q, trim_sol.state.r) };
        const dynamics::EulerAngles trim_eul{ Eigen::Vector3d(0.0, trim_sol.state.theta, trim_sol.state.phi) };
        const dynamics::EulerAngleRates trim_eul_dot = dynamics::_wB_BI_to_eul_dot(trim_w, trim_eul);
        std::ostringstream out;
        out << "trim_sol.converged: " << trim_sol.converged << "\n";
        out << "trim_sol.iterations: " << trim_sol.iterations << "\n";
        out << "trim_sol.residual_norm_2: " << trim_sol.residual_norm_2 << "\n";
        out << "trim_sol.residual_norm_inf: " << trim_sol.residual_norm_inf << "\n";
        out << "trim_sol.state.vB_BN: [" << trim_sol.state.vx << ", " << trim_sol.state.vy << ", " << trim_sol.state.vz << "]\n";
        out << "trim_sol.state.wB_BN: [" << trim_sol.state.p << ", " << trim_sol.state.q << ", " << trim_sol.state.r << "]\n";
        out << "trim_sol.state.phi_deg: " << global::rad_to_deg(trim_sol.state.phi) << "\n";
        out << "trim_sol.state.theta_deg: " << global::rad_to_deg(trim_sol.state.theta) << "\n";
        out << "trim_sol.ads: [Vinf=" << trim_sol_ads.Vinf
            << ", alpha_deg=" << global::rad_to_deg(trim_sol_ads.alpha)
            << ", beta_deg=" << global::rad_to_deg(trim_sol_ads.beta) << "]\n";
        out << "trim_sol.state.euler_dot_deg_s: ["
            << global::rad_to_deg(trim_eul_dot.phi_dot()) << ", "
            << global::rad_to_deg(trim_eul_dot.theta_dot()) << ", "
            << global::rad_to_deg(trim_eul_dot.psi_dot()) << "]\n";
        out << "trim_sol.input.elevator_deg: " << global::rad_to_deg(trim_sol.input.elevator) << "\n";
        out << "trim_sol.input.aileron_deg: " << global::rad_to_deg(trim_sol.input.aileron) << "\n";
        out << "trim_sol.input.rudder_deg: " << global::rad_to_deg(trim_sol.input.rudder) << "\n";
        out << "trim_sol.residual:\n"
            << "vx_dot: " << trim_sol.residual.vx_dot << "\n"
            << "vy_dot: " << trim_sol.residual.vy_dot << "\n"
            << "vz_dot: " << trim_sol.residual.vz_dot << "\n"
            << "p_dot: " << trim_sol.residual.p_dot << "\n"
            << "q_dot: " << trim_sol.residual.q_dot << "\n"
            << "r_dot: " << trim_sol.residual.r_dot << "\n"
            << "phi_dot: " << trim_sol.residual.phi_dot << "\n"
            << "theta_dot: " << trim_sol.residual.theta_dot << "\n"
            << "beta_error: " << trim_sol.residual.beta_error << "\n"
            << "phi_error: " << trim_sol.residual.phi_error << "\n"
            << "theta_error: " << trim_sol.residual.theta_error << "\n";
        return out.str();
    }


    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& xN_t, const TrimSolution& trim_sol) {
            dynamics::EulerAngles eul_curr;
            eul_curr.set(xN_t.q);
            dynamics::EulerAngles eul_trim{ Eigen::Vector3d(eul_curr.psi(), trim_sol.state.theta, trim_sol.state.phi) };
            dynamics::OrientationQuaternion qNB_trim;
            qNB_trim.set(eul_trim);

            dynamics::RigidBodyState xN_t_trim = { 
                .p = xN_t.p, 
                .v = dynamics::LinearVelocity{ Eigen::Vector3d(trim_sol.state.vx, trim_sol.state.vy, trim_sol.state.vz) },
                .q = qNB_trim,
                .w = dynamics::AngularVelocity{ Eigen::Vector3d(trim_sol.state.p, trim_sol.state.q, trim_sol.state.r) },
            };

            aerodynamics::AerodynamicState ads_trim = aerodynamics::compute_aerodynamic_state(xN_t_trim, trim_sol.conditions.windB);

        return { xN_t_trim, ads_trim };
    }

}
