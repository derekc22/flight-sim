#include <algorithm>
#include <cmath>
#include <sstream>
#include <iostream>
#include <string>
#include <utility> // For std::pair
#include <stdexcept>
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/trim/shared.hpp"
#include "simulation/util/cppad.hpp"
#include "simulation/util/util.hpp"
#include "simulation/vehicles/vehicles.hpp"

namespace trim {

    double send_control_to_solver_space(double u, double limit_min, double limit_max) {
        double mid = 0.5 * (limit_max + limit_min);
        double half_range = 0.5 * (limit_max - limit_min);

        if (half_range <= 0.0) {
            return 0.0;
        }

        double ratio = util::clamp_inside_1((u - mid) / half_range);
        return ratio / std::sqrt(std::max(1.0 - ratio * ratio, constants::eps));
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

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind) {
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_properties.propulsor_actuators;
        const actuators::ActuatorLimits_T<double> actuator_limits = actuators::pack_actuator_limits(aircraft.actuator_properties.surface_actuators, propulsor_actuators);
        const aerodynamics::AerodynamicState target_ads = aerodynamics::aerodynamic_state(aircraft.FRDFrameNED, wind);

        const TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators,
            .actuator_limits = actuator_limits,
            .fixed_actuator_inputs = actuators::FixedActuatorInputs_T{
                .flap = aircraft.operating_properties.fixed_actuator_inputs.flap,
                .spoiler = aircraft.operating_properties.fixed_actuator_inputs.spoiler,
            },
        };

        const TrimProblem<double> problem{
            .target = TrimTarget{
                .beta = target_ads.beta.data,
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
                .vx = aircraft.FRDFrameNED.vB_BN.data(0),
                .vz = aircraft.FRDFrameNED.vB_BN.data(2),
                .psi_dot = aircraft.FRDFrameNED.eulNB_dot.psi_dot()
            },
            .conditions = TrimConditions{
                .static_atm_state = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF),
                .windB = wind,
            },
            .state_guess = dynamics::State_T<double>{
                .vx = aircraft.FRDFrameNED.vB_BN.data.x(),
                .vy = aircraft.FRDFrameNED.vB_BN.data.y(),
                .vz = aircraft.FRDFrameNED.vB_BN.data.z(),
                .p = aircraft.FRDFrameNED.wB_BN.p(),
                .q = aircraft.FRDFrameNED.wB_BN.q(),
                .r = aircraft.FRDFrameNED.wB_BN.r(),
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
            },
            .input_guess = actuators::ActuatorInputs_T<double>{
                .elevator_cmd = 0,
                .aileron_cmd = 0,
                .rudder_cmd = 0,
                .front_propulsor_cmd = 0.5 * (actuator_limits.limit_max.front_propulsor_cmd + actuator_limits.limit_min.front_propulsor_cmd),
                .left_propulsor_cmd = 0.5 * (actuator_limits.limit_max.left_propulsor_cmd + actuator_limits.limit_min.left_propulsor_cmd),
                .right_propulsor_cmd = 0.5 * (actuator_limits.limit_max.right_propulsor_cmd + actuator_limits.limit_min.right_propulsor_cmd)
            }
        };

        const TrimSolution trim_sol = solve_trim(problem, model);

        if (!trim_sol.converged) {
            std::cout << print_trim_solution(trim_sol) << std::endl;
            throw std::runtime_error("trim::inspect_trim: Error, trim failed to converge");
        }

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

        constexpr const char* section_rule = "---------------------------";
        std::ostringstream out;
        out << "trim_sol.converged: " << trim_sol.converged << "\n";
        out << "trim_sol.iterations: " << trim_sol.iterations << "\n\n";

        // out << "trim_sol.residual_norm_2: " << trim_sol.residual_norm_2 << "\n";
        // out << "trim_sol.residual_norm_inf: " << trim_sol.residual_norm_inf << "\n\n";

        out << "trim_sol.weighted_residual_norm_2: " << trim_sol.weighted_residual_norm_2 << "\n";
        out << "trim_sol.weighted_residual_norm_inf: " << trim_sol.weighted_residual_norm_inf << "\n\n";

        out << "trim_sol.state:\n" << section_rule << "\n";
        out << "vB_BN: [" << trim_sol.state.vx << ", " << trim_sol.state.vy << ", " << trim_sol.state.vz << "]\n";
        out << "wB_BN: [" << trim_sol.state.p << ", " << trim_sol.state.q << ", " << trim_sol.state.r << "]\n";
        out << "eulNB: [n/a, "
            << trim_sol.state.theta << ", "
            << trim_sol.state.phi << "]\n";
        out << "eulNB_dot: ["
            << trim_eul_dot.phi_dot() << ", "
            << trim_eul_dot.theta_dot() << ", "
            << trim_eul_dot.psi_dot() << "]\n\n";

        out << "eulNB_deg: [n/a, "
            << util::rad_to_deg(trim_sol.state.theta) << ", "
            << util::rad_to_deg(trim_sol.state.phi) << "]\n";
        out << "eulNB_dot_deg_s: ["
            << util::rad_to_deg(trim_eul_dot.phi_dot()) << ", "
            << util::rad_to_deg(trim_eul_dot.theta_dot()) << ", "
            << util::rad_to_deg(trim_eul_dot.psi_dot()) << "]\n\n";

        out << "trim_sol.ads:\n"
            << section_rule << "\n"
            << "Vinf: " << trim_sol_ads.Vinf << "\n"
            << "alpha_deg: " << util::rad_to_deg(trim_sol_ads.alpha) << "\n"
            << "beta_deg: " << util::rad_to_deg(trim_sol_ads.beta) << "\n\n";

        out << "trim_sol.input:\n" << section_rule << "\n";
        out << "elevator_cmd_deg: " << util::rad_to_deg(trim_sol.input.elevator_cmd) << "\n";
        out << "aileron_cmd_deg: " << util::rad_to_deg(trim_sol.input.aileron_cmd) << "\n";
        out << "rudder_cmd_deg: " << util::rad_to_deg(trim_sol.input.rudder_cmd) << "\n";
        out << "front_propulsor_cmd: " << trim_sol.input.front_propulsor_cmd << "\n";
        out << "left_propulsor_cmd: " << trim_sol.input.left_propulsor_cmd << "\n";
        out << "right_propulsor_cmd: " << trim_sol.input.right_propulsor_cmd << "\n\n";

        out << "trim_sol.wrench:\n" << section_rule << "\n";
        out << "F: [" << trim_sol.wrench.F.data.x() << ", " << trim_sol.wrench.F.data.y() << ", " << trim_sol.wrench.F.data.z() << "]\n";
        out << "M: [" << trim_sol.wrench.M.data.x() << ", " << trim_sol.wrench.M.data.y() << ", " << trim_sol.wrench.M.data.z() << "]\n\n";

        out << "trim_sol.residual:\n"
            << section_rule << "\n"
            << "vx_dot: " << trim_sol.residual.vx_dot << "\n"
            << "vy_dot: " << trim_sol.residual.vy_dot << "\n"
            << "vz_dot: " << trim_sol.residual.vz_dot << "\n"
            << "p_dot: " << trim_sol.residual.p_dot << "\n"
            << "q_dot: " << trim_sol.residual.q_dot << "\n"
            << "r_dot: " << trim_sol.residual.r_dot << "\n"
            << "phi_dot: " << trim_sol.residual.phi_dot << "\n"
            << "theta_dot: " << trim_sol.residual.theta_dot << "\n"
            << "beta_err: " << trim_sol.residual.beta_err << "\n"
            << "phi_err: " << trim_sol.residual.phi_err << "\n"
            << "theta_err: " << trim_sol.residual.theta_err << "\n"
            << "vx_err: " << trim_sol.residual.vx_err << "\n"
            << "vz_err: " << trim_sol.residual.vz_err << "\n"
            << "psi_dot_err: " << trim_sol.residual.psi_dot_err << "\n\n";

        out << "trim_sol.weighted_residual:\n"
            << section_rule << "\n"
            << "vx_dot: " << trim_sol.weighted_residual.vx_dot << "\n"
            << "vy_dot: " << trim_sol.weighted_residual.vy_dot << "\n"
            << "vz_dot: " << trim_sol.weighted_residual.vz_dot << "\n"
            << "p_dot: " << trim_sol.weighted_residual.p_dot << "\n"
            << "q_dot: " << trim_sol.weighted_residual.q_dot << "\n"
            << "r_dot: " << trim_sol.weighted_residual.r_dot << "\n"
            << "phi_dot: " << trim_sol.weighted_residual.phi_dot << "\n"
            << "theta_dot: " << trim_sol.weighted_residual.theta_dot << "\n"
            << "beta_err: " << trim_sol.weighted_residual.beta_err << "\n"
            << "phi_err: " << trim_sol.weighted_residual.phi_err << "\n"
            << "theta_err: " << trim_sol.weighted_residual.theta_err << "\n"
            << "vx_err: " << trim_sol.weighted_residual.vx_err << "\n"
            << "vz_err: " << trim_sol.weighted_residual.vz_err << "\n"
            << "psi_dot_err: " << trim_sol.weighted_residual.psi_dot_err << "\n";
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

            aerodynamics::AerodynamicState ads_t_trim = aerodynamics::compute_aerodynamic_state(xN_t_trim, trim_sol.conditions.windB);

        return { xN_t_trim, ads_t_trim };
    }

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol){
        actuators::SurfaceActuatorInputs_T<double> surface_actuator_cmd_trim{
            .aileron_cmd = trim_sol.input.aileron_cmd,
            .elevator_cmd = trim_sol.input.elevator_cmd,
            .rudder_cmd = trim_sol.input.rudder_cmd,
        };
        actuators::PropulsorActuatorInputs_T<double> propulsor_actuator_cmd_trim{
            .front_propulsor_cmd = trim_sol.input.front_propulsor_cmd,
            .left_propulsor_cmd = trim_sol.input.left_propulsor_cmd,
            .right_propulsor_cmd = trim_sol.input.right_propulsor_cmd,
        };

        return { surface_actuator_cmd_trim,  propulsor_actuator_cmd_trim };
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
