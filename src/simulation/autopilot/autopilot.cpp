#include "simulation/autopilot/autopilot.hpp"
#include <cppad/cppad.hpp>
#include <iostream>
#include <stdexcept>


namespace autopilot { // to encompass autonomy and trim

    static Eigen::Matrix<double, trim_variable_dofs, 1> _trim_variable_vector(const std::vector<double>& z) {
        if (z.size() != trim_variable_dofs) {
            throw std::invalid_argument("autopilot::_trim_variable_vector: trim variable vector has incorrect size");
        }

        Eigen::Matrix<double, trim_variable_dofs, 1> out;
        for (std::size_t i = 0; i < trim_variable_dofs; ++i) {
            out(static_cast<Eigen::Index>(i)) = z[i];
        }
        return out;
    }

    static Eigen::Matrix<double, trim_residual_dofs, 1> _trim_residual_vector(const std::vector<double>& residual) {
        if (residual.size() != trim_residual_dofs) {
            throw std::invalid_argument("autopilot::_trim_residual_vector: trim residual vector has incorrect size");
        }

        Eigen::Matrix<double, trim_residual_dofs, 1> out;
        for (std::size_t i = 0; i < trim_residual_dofs; ++i) {
            out(static_cast<Eigen::Index>(i)) = residual[i];
        }
        return out;
    }

    static std::vector<double> _std_vector(const Eigen::Matrix<double, trim_variable_dofs, 1>& z) {
        return std::vector<double>(z.data(), z.data() + trim_variable_dofs);
    }

    static double _control_solver_variable_from_physical(double u, double limit) {
        if (limit <= 0.0) {
            return 0.0;
        }

        constexpr double ratio_eps = 1e-9;
        const double ratio = std::clamp(u / limit, -1.0 + ratio_eps, 1.0 - ratio_eps);
        return ratio / std::sqrt(std::max(1.0 - ratio * ratio, ratio_eps));
    }

    template <typename T>
    static T _control_from_solver_variable_T(const T& u_solver, double limit) {
        if (limit <= 0.0) {
            return T(0);
        }

        return T(limit) * u_solver / global::sqrt(T(1) + u_solver * u_solver);
    }

    static std::vector<double> _pack_trim_solver_variables(const TrimState<double>& x, const TrimInput<double>& u, const control::ControlSurfaceLimits& limits) {
        std::vector<double> z = pack_trim_variables_T<double>(x, u);
        z[8] = _control_solver_variable_from_physical(u.elevator, limits.elevator_max);
        z[9] = _control_solver_variable_from_physical(u.aileron, limits.aileron_max);
        z[10] = _control_solver_variable_from_physical(u.rudder, limits.rudder_max);
        return z;
    }

    template <typename T>
    static TrimInput<T> _unpack_trim_solver_input_T(const std::vector<T>& z, const control::ControlSurfaceLimits& limits) {
        _check_trim_variables_size(z.size());
        return TrimInput<T>{
            .elevator = _control_from_solver_variable_T<T>(z[8], limits.elevator_max),
            .aileron = _control_from_solver_variable_T<T>(z[9], limits.aileron_max),
            .rudder = _control_from_solver_variable_T<T>(z[10], limits.rudder_max),
        };
    }

    template <typename T>
    static std::vector<T> _evaluate_trim_solver_residual_vector_T(const std::vector<T>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        const TrimState<T> x = unpack_trim_state_T<T>(z);
        const TrimInput<T> u = _unpack_trim_solver_input_T<T>(z, model.control.limits);
        const TrimResidual<T> residual = evaluate_trim_residual<T>(x, u, model, target, conditions);
        return pack_trim_residual_T<T>(residual);
    }

    static std::vector<double> _evaluate_trim_solver_residual_vector(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        return _evaluate_trim_solver_residual_vector_T<double>(z, model, target, conditions);
    }

    static std::vector<double> _compute_trim_solver_residual_jac(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        if (z.size() != trim_variable_dofs) {
            throw std::invalid_argument("autopilot::_compute_trim_solver_residual_jac: trim variable vector has incorrect size");
        }

        std::vector<CppAD::AD<double>> z_T(trim_variable_dofs);
        for (std::size_t i = 0; i < trim_variable_dofs; ++i) {
            z_T[i] = z[i];
        }

        CppAD::Independent(z_T);
        const std::vector<CppAD::AD<double>> y_T = _evaluate_trim_solver_residual_vector_T<CppAD::AD<double>>(z_T, model, target, conditions);
        CppAD::ADFun<double> f(z_T, y_T);
        return f.Jacobian(z);
    }

    static Eigen::Matrix<double, trim_residual_dofs, 1> _trim_residual_weights(const TrimSolveOptions& options) {
        Eigen::Matrix<double, trim_residual_dofs, 1> w;
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

    static double _residual_norm_inf(const Eigen::Matrix<double, trim_residual_dofs, 1>& residual) {
        return residual.cwiseAbs().maxCoeff();
    }

    static void _validate_trim_solve_options(const TrimSolveOptions& options) {
        if (options.residual_tolerance < 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: residual_tolerance must be nonnegative");
        }

        if (options.step_tolerance < 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: step_tolerance must be nonnegative");
        }

        if (options.initial_damping < 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: initial_damping must be nonnegative");
        }

        if (options.damping_growth <= 1.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: damping_growth must be greater than 1");
        }

        if (options.linear_accel_scale <= 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: linear_accel_scale must be positive");
        }

        if (options.angular_accel_scale <= 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: angular_accel_scale must be positive");
        }

        if (options.angle_rate_scale <= 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: angle_rate_scale must be positive");
        }

        if (options.angle_error_scale <= 0.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: angle_error_scale must be positive");
        }

        if (options.backtrack_scale <= 0.0 || options.backtrack_scale >= 1.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: backtrack_scale must be in (0, 1)");
        }

        if (options.min_step_scale <= 0.0 || options.min_step_scale > 1.0) {
            throw std::invalid_argument("autopilot::_validate_trim_solve_options: min_step_scale must be in (0, 1]");
        }
    }

    static TrimSolution _trim_solution(const std::vector<double>& z, const std::vector<double>& residual, const TrimModelContext& model, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.state = unpack_trim_state_T<double>(z);
        out.input = _unpack_trim_solver_input_T<double>(z, model.control.limits);
        out.variables = pack_trim_variables_T<double>(out.state, out.input);
        out.converged = converged;
        out.iterations = iterations;

        const Eigen::Matrix<double, trim_residual_dofs, 1> residual_vec = _trim_residual_vector(residual);
        out.residual = TrimResidual<double>{
            .vx_dot = residual[0],
            .vy_dot = residual[1],
            .vz_dot = residual[2],
            .p_dot = residual[3],
            .q_dot = residual[4],
            .r_dot = residual[5],
            .phi_dot = residual[6],
            .theta_dot = residual[7],
            .beta_error = residual[8],
            .phi_error = residual[9],
            .theta_error = residual[10],
        };
        out.residual_norm_2 = residual_vec.norm();
        out.residual_norm_inf = _residual_norm_inf(residual_vec);
        return out;
    }

    std::vector<double> evaluate_trim_residual_vector(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        return evaluate_trim_residual_vector_T<double>(z, model, target, conditions);
    }

    std::vector<double> compute_trim_residual_jac(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        if (z.size() != trim_variable_dofs) {
            throw std::invalid_argument("autopilot::compute_trim_residual_jac: trim variable vector has incorrect size");
        }

        std::vector<CppAD::AD<double>> z_T(trim_variable_dofs);
        for (std::size_t i = 0; i < trim_variable_dofs; ++i) {
            z_T[i] = z[i];
        }

        CppAD::Independent(z_T);
        const std::vector<CppAD::AD<double>> y_T = evaluate_trim_residual_vector_T<CppAD::AD<double>>(z_T, model, target, conditions);
        CppAD::ADFun<double> f(z_T, y_T);
        return f.Jacobian(z);
    }

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModelContext& model, TrimSolveOptions options) {
        _validate_trim_solve_options(options);

        std::vector<double> z = _pack_trim_solver_variables(problem.state_guess, problem.input_guess, model.control.limits);

        std::vector<double> residual = _evaluate_trim_solver_residual_vector(z, model, problem.target, problem.conditions);
        double damping = options.initial_damping;
        std::size_t iterations_completed = 0;
        const Eigen::Matrix<double, trim_residual_dofs, 1> weights = _trim_residual_weights(options);

        for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
            iterations_completed = iteration;
            const Eigen::Matrix<double, trim_residual_dofs, 1> residual_vec = _trim_residual_vector(residual);
            const Eigen::Matrix<double, trim_residual_dofs, 1> weighted_residual_vec = weights.cwiseProduct(residual_vec);
            const double residual_norm_inf = _residual_norm_inf(residual_vec);

            if (residual_norm_inf <= options.residual_tolerance) {
                return _trim_solution(z, residual, model, true, iteration);
            }

            const std::vector<double> jac_flat = _compute_trim_solver_residual_jac(z, model, problem.target, problem.conditions);
            const Eigen::Map<const Eigen::Matrix<double, trim_residual_dofs, trim_variable_dofs, Eigen::RowMajor>> jac_raw_map(jac_flat.data());
            const Eigen::Matrix<double, trim_residual_dofs, trim_variable_dofs> jac = weights.asDiagonal() * Eigen::Matrix<double, trim_residual_dofs, trim_variable_dofs>(jac_raw_map);

            const Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs> hess = jac.transpose() * jac + damping * Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs>::Identity();
            const Eigen::Matrix<double, trim_variable_dofs, 1> gradient = jac.transpose() * weighted_residual_vec;
            const Eigen::LDLT<Eigen::Matrix<double, trim_variable_dofs, trim_variable_dofs>> ldlt(hess);
            const Eigen::Matrix<double, trim_variable_dofs, 1> step = ldlt.solve(-gradient);

            if (ldlt.info() != Eigen::Success || !step.allFinite()) {
                break;
            }

            if (step.norm() <= options.step_tolerance) {
                return _trim_solution(z, residual, model, residual_norm_inf <= options.residual_tolerance, iteration);
            }

            bool accepted = false;
            double step_scale = 1.0;
            const double residual_norm_2 = weighted_residual_vec.norm();

            while (step_scale >= options.min_step_scale) {
                const Eigen::Matrix<double, trim_variable_dofs, 1> z_trial_vec = _trim_variable_vector(z) + step_scale * step;
                const std::vector<double> z_trial = _std_vector(z_trial_vec);
                const std::vector<double> residual_trial = _evaluate_trim_solver_residual_vector(z_trial, model, problem.target, problem.conditions);
                const double residual_trial_norm_2 = weights.cwiseProduct(_trim_residual_vector(residual_trial)).norm();

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

        return _trim_solution(z, residual, model, false, iterations_completed);
    }

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind) {

        const TrimModelContext model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .control = aircraft.control_properties,
            .fixed_controls = TrimFixedControls{
                .flap = 0.0,
                .spoiler = 0.0,
            },
        };

        const TrimProblem<double> problem {
            .target = TrimTarget {
                .beta = aircraft.aerodynamicState(aircraft.FRDFrameNED, wind).beta.data,
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
            },
            .conditions = TrimConditions {
                .rho = aircraft.atmosphericState(aircraft.FRDFrameECEF).rho,
                .windB = wind,
            },
            .state_guess = TrimState<double> {
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

        const TrimSolution trim = solve_trim(problem, model);
        dynamics::Twist_T<double> trim_twist;
        trim_twist.v << trim.state.vx, trim.state.vy, trim.state.vz;
        trim_twist.w << trim.state.p, trim.state.q, trim.state.r;
        const aerodynamics::AerodynamicState_T<double> trim_ads = aerodynamics::compute_aerodynamic_state_T<double>(trim_twist, wind);
        const dynamics::AngularVelocity trim_w{ Eigen::Vector3d(trim.state.p, trim.state.q, trim.state.r) };
        const dynamics::EulerAngles trim_eul{ Eigen::Vector3d(0.0, trim.state.theta, trim.state.phi) };
        const dynamics::EulerAngleRates trim_eul_dot = dynamics::_wB_BI2eul_dot(trim_w, trim_eul);

        std::cout << "trim.converged: " << trim.converged << "\n";
        // std::cout << "trim.status: " << trim_status << "\n";
        std::cout << "trim.iterations: " << trim.iterations << "\n";
        std::cout << "trim.residual_norm_2: " << trim.residual_norm_2 << "\n";
        std::cout << "trim.residual_norm_inf: " << trim.residual_norm_inf << "\n";
        std::cout << "trim.state.vB_BN: [" << trim.state.vx << ", " << trim.state.vy << ", " << trim.state.vz << "]\n";
        std::cout << "trim.state.wB_BN: [" << trim.state.p << ", " << trim.state.q << ", " << trim.state.r << "]\n";
        std::cout << "trim.state.phi_deg: " << global::rad_to_deg(trim.state.phi) << "\n";
        std::cout << "trim.state.theta_deg: " << global::rad_to_deg(trim.state.theta) << "\n";
        std::cout << "trim.ads: [Vinf=" << trim_ads.Vinf
                  << ", alpha_deg=" << global::rad_to_deg(trim_ads.alpha)
                  << ", beta_deg=" << global::rad_to_deg(trim_ads.beta) << "]\n";
        std::cout << "trim.state.euler_dot_deg_s: ["
                  << global::rad_to_deg(trim_eul_dot.phi_dot()) << ", "
                  << global::rad_to_deg(trim_eul_dot.theta_dot()) << ", "
                  << global::rad_to_deg(trim_eul_dot.psi_dot()) << "]\n";
        std::cout << "trim.input.elevator_deg: " << global::rad_to_deg(trim.input.elevator) << "\n";
        std::cout << "trim.input.aileron_deg: " << global::rad_to_deg(trim.input.aileron) << "\n";
        std::cout << "trim.input.rudder_deg: " << global::rad_to_deg(trim.input.rudder) << "\n";
        std::cout << "trim.residual: ["
                  << trim.residual.vx_dot << ", "
                  << trim.residual.vy_dot << ", "
                  << trim.residual.vz_dot << ", "
                  << trim.residual.p_dot << ", "
                  << trim.residual.q_dot << ", "
                  << trim.residual.r_dot << ", "
                  << trim.residual.phi_dot << ", "
                  << trim.residual.theta_dot << ", "
                  << trim.residual.beta_error << ", "
                  << trim.residual.phi_error << ", "
                  << trim.residual.theta_error << "]\n";

        return trim;
    }


    
}
