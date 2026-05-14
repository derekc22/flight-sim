#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/util/public.hpp"
#include "simulation/vehicles/public.hpp"

namespace trim {

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
        const dynamics::EulerAngleRates trim_eul_dot = dynamics::wB_BI_to_eul_dot(trim_w, trim_eul);

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
                .v = dynamics::TranslationalVelocity{ Eigen::Vector3d(trim_sol.state.vx, trim_sol.state.vy, trim_sol.state.vz) },
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

}
