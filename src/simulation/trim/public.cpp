#include <Eigen/Dense>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <spdlog/spdlog.h>
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/trim/private.hpp"
#include "simulation/util/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, autodiff::AutoDiffModel& model, const atmospheric::Wind& wind) {

        const actuators::ActuatorLimits_T actuator_limits = model.actuator_limits;
        const actuators::ActuatorInputs_T actuator_limits_max = model.actuator_limits.limit_max;
        const actuators::ActuatorInputs_T actuator_limits_min = model.actuator_limits.limit_min;
        const aerodynamics::AerodynamicState target_aero = aerodynamics::compute_aerodynamic_state(aircraft.FRDFrameNED, wind);

        const TrimProblem<double> problem{
            .target = TrimTarget{
                .beta = target_aero.beta.data,
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
                .vx = aircraft.FRDFrameNED.vB_BN.data(0),
                .vz = aircraft.FRDFrameNED.vB_BN.data(2),
                .psi_dot = aircraft.FRDFrameNED.eulNB_dot.psi_dot()
            },
            .conditions = operating::OperatingConditions{
                .atm = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF),
                .windB = wind,
                .steady_state = true
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
                .front_propulsor_cmd = 0.5 * (actuator_limits_max.front_propulsor_cmd + actuator_limits_min.front_propulsor_cmd),
                .left_propulsor_cmd = 0.5 * (actuator_limits_max.left_propulsor_cmd + actuator_limits_min.left_propulsor_cmd),
                .right_propulsor_cmd = 0.5 * (actuator_limits_max.right_propulsor_cmd + actuator_limits_min.right_propulsor_cmd)
            }
        };

        const TrimSolution trim_sol = solve_trim(problem, model);

        if (!trim_sol.converged) {
            spdlog::error(print_trim_solution(trim_sol));
            throw std::runtime_error("trim::inspect_trim: Error, trim failed to converge");
        }

        return trim_sol;
    }


    std::string print_trim_solution(const TrimSolution& trim_sol) {
        dynamics::Twist_T<double> trim_sol_twist;
        trim_sol_twist.v << trim_sol.operating_point.state.vx, trim_sol.operating_point.state.vy, trim_sol.operating_point.state.vz;
        trim_sol_twist.w << trim_sol.operating_point.state.p, trim_sol.operating_point.state.q, trim_sol.operating_point.state.r;

        const aerodynamics::AerodynamicState_T<double> trim_sol_aero = aerodynamics::compute_aerodynamic_state_T<double>(trim_sol_twist, trim_sol.conditions.windB);

        const dynamics::AngularVelocity trim_w{ Eigen::Vector3d(trim_sol.operating_point.state.p, trim_sol.operating_point.state.q, trim_sol.operating_point.state.r) };
        const dynamics::EulerAngles trim_eul{ Eigen::Vector3d(0.0, trim_sol.operating_point.state.theta, trim_sol.operating_point.state.phi) };
        const dynamics::EulerAngleRates trim_eul_dot = dynamics::wB_BI_to_eul_dot(trim_w, trim_eul);

        constexpr const char* section_rule = "------------------------------------------------------";

        std::ostringstream out;

        out << "\n" << "summary:\n" << section_rule << "\n";
        out << "trim_sol.iterations: " << trim_sol.iterations << "\n\n";

        // out << "trim_sol.residual_norm_2: " << trim_sol.residual_norm_2 << "\n";
        // out << "trim_sol.residual_norm_inf: " << trim_sol.residual_norm_inf << "\n\n";

        out << "trim_sol.weighted_residual_norm_2: " << trim_sol.weighted_residual_norm_2 << "\n";
        out << "trim_sol.weighted_residual_norm_inf: " << trim_sol.weighted_residual_norm_inf << "\n\n";

        out << "trim_sol.operating_point.state:\n" << section_rule << "\n";
        out << "vB_BN: [" << trim_sol.operating_point.state.vx << ", " << trim_sol.operating_point.state.vy << ", " << trim_sol.operating_point.state.vz << "]\n";
        out << "wB_BN: [" << trim_sol.operating_point.state.p << ", " << trim_sol.operating_point.state.q << ", " << trim_sol.operating_point.state.r << "]\n";
        out << "eulNB: [n/a, "
            << trim_sol.operating_point.state.theta << ", "
            << trim_sol.operating_point.state.phi << "]\n";
        out << "eulNB_dot: ["
            << trim_eul_dot.phi_dot() << ", "
            << trim_eul_dot.theta_dot() << ", "
            << trim_eul_dot.psi_dot() << "]\n\n";

        out << "eulNB_deg: [n/a, "
            << util::rad_to_deg(trim_sol.operating_point.state.theta) << ", "
            << util::rad_to_deg(trim_sol.operating_point.state.phi) << "]\n";
        out << "eulNB_dot_deg_s: ["
            << util::rad_to_deg(trim_eul_dot.phi_dot()) << ", "
            << util::rad_to_deg(trim_eul_dot.theta_dot()) << ", "
            << util::rad_to_deg(trim_eul_dot.psi_dot()) << "]\n\n";

        out << "trim_sol.aero:\n"
            << section_rule << "\n"
            << "Vinf: " << trim_sol_aero.Vinf << "\n"
            << "alpha_deg: " << util::rad_to_deg(trim_sol_aero.alpha) << "\n"
            << "beta_deg: " << util::rad_to_deg(trim_sol_aero.beta) << "\n\n";

        out << "trim_sol.operating_point.input:\n" << section_rule << "\n";
        out << "elevator_cmd_deg: " << util::rad_to_deg(trim_sol.operating_point.input.elevator_cmd) << "\n";
        out << "aileron_cmd_deg: " << util::rad_to_deg(trim_sol.operating_point.input.aileron_cmd) << "\n";
        out << "rudder_cmd_deg: " << util::rad_to_deg(trim_sol.operating_point.input.rudder_cmd) << "\n";
        out << "front_propulsor_cmd: " << trim_sol.operating_point.input.front_propulsor_cmd << "\n";
        out << "left_propulsor_cmd: " << trim_sol.operating_point.input.left_propulsor_cmd << "\n";
        out << "right_propulsor_cmd: " << trim_sol.operating_point.input.right_propulsor_cmd << "\n\n";

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


    std::tuple<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& Xt, const TrimSolution& trim_sol) {
            dynamics::EulerAngles eul_curr;
            eul_curr.set(Xt.q);
            dynamics::EulerAngles eul_trim{ Eigen::Vector3d(eul_curr.psi(), trim_sol.operating_point.state.theta, trim_sol.operating_point.state.phi) };
            dynamics::OrientationQuaternion qNB_trim;
            qNB_trim.set(eul_trim);

            dynamics::RigidBodyState Xt_trim = {
                .p = Xt.p,
                .v = dynamics::TranslationalVelocity{ Eigen::Vector3d(trim_sol.operating_point.state.vx, trim_sol.operating_point.state.vy, trim_sol.operating_point.state.vz) },
                .q = qNB_trim,
                .w = dynamics::AngularVelocity{ Eigen::Vector3d(trim_sol.operating_point.state.p, trim_sol.operating_point.state.q, trim_sol.operating_point.state.r) },
            };

            aerodynamics::AerodynamicState aero_t_trim = aerodynamics::compute_aerodynamic_state(Xt_trim, trim_sol.conditions.windB);

        return { Xt_trim, aero_t_trim };
    }

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol) {
        actuators::SurfaceActuatorInputs_T<double> surface_actuator_cmd_trim{
            .elevator_cmd = trim_sol.operating_point.input.elevator_cmd,
            .aileron_cmd = trim_sol.operating_point.input.aileron_cmd,
            .rudder_cmd = trim_sol.operating_point.input.rudder_cmd,
        };
        actuators::PropulsorActuatorInputs_T<double> propulsor_actuator_cmd_trim{
            .front_propulsor_cmd = trim_sol.operating_point.input.front_propulsor_cmd,
            .left_propulsor_cmd = trim_sol.operating_point.input.left_propulsor_cmd,
            .right_propulsor_cmd = trim_sol.operating_point.input.right_propulsor_cmd,
        };

        return { surface_actuator_cmd_trim,  propulsor_actuator_cmd_trim };
    }

    /** @deprecated */
    // void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol) {
    //     surface_actuators.aileron.prev_cmd = trim_sol.operating_point.input.aileron_cmd;
    //     surface_actuators.elevator.prev_cmd = trim_sol.operating_point.input.elevator_cmd;
    //     surface_actuators.rudder.prev_cmd = trim_sol.operating_point.input.rudder_cmd;

    //     propulsor_actuators.front_propulsor.prev_cmd = trim_sol.operating_point.input.front_propulsor_cmd;
    //     propulsor_actuators.left_propulsor.prev_cmd = trim_sol.operating_point.input.left_propulsor_cmd;
    //     propulsor_actuators.right_propulsor.prev_cmd = trim_sol.operating_point.input.right_propulsor_cmd;
    // }

    std::tuple<actuators::SurfaceActuatorInputs_T<double>, actuators::PropulsorActuatorInputs_T<double>> update_actuators_from_trim(actuators::SurfaceActuatorInputs_T<double>& surface_actuator_inputs, actuators::PropulsorActuatorInputs_T<double>& propulsor_actuator_inputs, const TrimSolution& trim_sol) {
        surface_actuator_inputs.elevator_cmd = trim_sol.operating_point.input.elevator_cmd;
        surface_actuator_inputs.aileron_cmd = trim_sol.operating_point.input.aileron_cmd;
        surface_actuator_inputs.rudder_cmd = trim_sol.operating_point.input.rudder_cmd;
        propulsor_actuator_inputs.front_propulsor_cmd = trim_sol.operating_point.input.front_propulsor_cmd;
        propulsor_actuator_inputs.left_propulsor_cmd = trim_sol.operating_point.input.left_propulsor_cmd;
        propulsor_actuator_inputs.right_propulsor_cmd = trim_sol.operating_point.input.right_propulsor_cmd;

        return { surface_actuator_inputs, propulsor_actuator_inputs };
    }

}
