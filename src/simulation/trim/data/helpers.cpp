#include <Eigen/Dense>
#include <sstream>
#include "simulation/aerodynamics/public/detail/state.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/trim/public/data/helpers.hpp"
#include "simulation/util/public/trig.hpp"

namespace trim {

	std::string print_trim_solution(
	    const TrimSolution& trim_sol)
	{
		const operating::OperatingPoint_T<double>& operating_point = trim_sol.operating_point;
		const dynamics::State_T<double>& state = operating_point.state;

		const dynamics::Wrench_T<double>& wrench = trim_sol.wrench;
		const Eigen::Vector3d& F = wrench.F;
		const Eigen::Vector3d& M = wrench.M;

		const TrimResidual_T<double>& residual = trim_sol.residual;
		const TrimResidual_T<double>& weighted_residual = trim_sol.weighted_residual;

		dynamics::Twist_T<double> trim_sol_twist;
		trim_sol_twist.v << state.vx, state.vy, state.vz;
		trim_sol_twist.w << state.p, state.q, state.r;

		const aerodynamics::AerodynamicState_T<double> trim_sol_aero =
		    aerodynamics::compute_aerodynamic_state_T<double>(trim_sol_twist, trim_sol.conditions.windB);

		const dynamics::AngularVelocity trim_w{Eigen::Vector3d(state.p, state.q, state.r)};
		const dynamics::EulerAngles trim_eul{Eigen::Vector3d(0.0, state.theta, state.phi)};
		const dynamics::EulerAngleRates trim_eul_dot = dynamics::wB_BI_to_eul_dot(trim_w, trim_eul);
		const actuators::SurfaceActuatorInputs_T<double>& surface_inputs = operating_point.input.surface_inputs;
		const actuators::PropulsorActuatorInputs_T<double>& propulsor_inputs = operating_point.input.propulsor_inputs;

		constexpr const char* section_rule = "------------------------------------------------------";

		std::ostringstream out;

		out << "\n" << "summary:\n" << section_rule << "\n";
		out << "trim_sol.iterations: " << trim_sol.iterations << "\n\n";

		// out << "trim_sol.residual_norm_2: " << trim_sol.residual_norm_2 << "\n";
		// out << "trim_sol.residual_norm_inf: " << trim_sol.residual_norm_inf << "\n\n";

		out << "trim_sol.weighted_residual_norm_2: " << trim_sol.weighted_residual_norm_2 << "\n";
		out << "trim_sol.weighted_residual_norm_inf: " << trim_sol.weighted_residual_norm_inf << "\n\n";

		out << "trim_sol.operating_point.state:\n" << section_rule << "\n";
		out << "vB_BN: [" << state.vx << ", " << state.vy << ", " << state.vz << "]\n";
		out << "wB_BN: [" << state.p << ", " << state.q << ", " << state.r << "]\n";
		out << "eulNB: [n/a, " << state.theta << ", " << state.phi << "]\n";
		out << "eulNB_dot: [" << trim_eul_dot.phi_dot() << ", " << trim_eul_dot.theta_dot() << ", "
		    << trim_eul_dot.psi_dot() << "]\n\n";

		out << "eulNB_deg: [n/a, " << util::rad_to_deg(state.theta) << ", " << util::rad_to_deg(state.phi) << "]\n";
		out << "eulNB_dot_deg_s: [" << util::rad_to_deg(trim_eul_dot.phi_dot()) << ", "
		    << util::rad_to_deg(trim_eul_dot.theta_dot()) << ", " << util::rad_to_deg(trim_eul_dot.psi_dot())
		    << "]\n\n";

		out << "trim_sol.aero:\n"
		    << section_rule << "\n"
		    << "Vinf: " << trim_sol_aero.Vinf << "\n"
		    << "alpha_deg: " << util::rad_to_deg(trim_sol_aero.alpha) << "\n"
		    << "beta_deg: " << util::rad_to_deg(trim_sol_aero.beta) << "\n\n";

		out << "trim_sol.operating_point.input:\n" << section_rule << "\n";
		out << "elevator_cmd_deg: " << util::rad_to_deg(surface_inputs.elevator_cmd) << "\n";
		out << "aileron_cmd_deg: " << util::rad_to_deg(surface_inputs.aileron_cmd) << "\n";
		out << "rudder_cmd_deg: " << util::rad_to_deg(surface_inputs.rudder_cmd) << "\n";
		out << "front_propulsor_cmd: " << propulsor_inputs.front_propulsor_cmd << "\n";
		out << "left_propulsor_cmd: " << propulsor_inputs.left_propulsor_cmd << "\n";
		out << "right_propulsor_cmd: " << propulsor_inputs.right_propulsor_cmd << "\n\n";

		out << "trim_sol.wrench:\n" << section_rule << "\n";
		out << "F: [" << F.x() << ", " << F.y() << ", " << F.z() << "]\n";
		out << "M: [" << M.x() << ", " << M.y() << ", " << M.z() << "]\n\n";

		out << "trim_sol.residual:\n"
		    << section_rule << "\n"
		    << "vx_dot: " << residual.vx_dot << "\n"
		    << "vy_dot: " << residual.vy_dot << "\n"
		    << "vz_dot: " << residual.vz_dot << "\n"
		    << "p_dot: " << residual.p_dot << "\n"
		    << "q_dot: " << residual.q_dot << "\n"
		    << "r_dot: " << residual.r_dot << "\n"
		    << "phi_dot: " << residual.phi_dot << "\n"
		    << "theta_dot: " << residual.theta_dot << "\n"
		    << "beta_err: " << residual.beta_err << "\n"
		    << "phi_err: " << residual.phi_err << "\n"
		    << "theta_err: " << residual.theta_err << "\n"
		    << "vx_err: " << residual.vx_err << "\n"
		    << "vz_err: " << residual.vz_err << "\n"
		    << "psi_dot_err: " << residual.psi_dot_err << "\n\n";

		out << "trim_sol.weighted_residual:\n"
		    << section_rule << "\n"
		    << "vx_dot: " << weighted_residual.vx_dot << "\n"
		    << "vy_dot: " << weighted_residual.vy_dot << "\n"
		    << "vz_dot: " << weighted_residual.vz_dot << "\n"
		    << "p_dot: " << weighted_residual.p_dot << "\n"
		    << "q_dot: " << weighted_residual.q_dot << "\n"
		    << "r_dot: " << weighted_residual.r_dot << "\n"
		    << "phi_dot: " << weighted_residual.phi_dot << "\n"
		    << "theta_dot: " << weighted_residual.theta_dot << "\n"
		    << "beta_err: " << weighted_residual.beta_err << "\n"
		    << "phi_err: " << weighted_residual.phi_err << "\n"
		    << "theta_err: " << weighted_residual.theta_err << "\n"
		    << "vx_err: " << weighted_residual.vx_err << "\n"
		    << "vz_err: " << weighted_residual.vz_err << "\n"
		    << "psi_dot_err: " << weighted_residual.psi_dot_err << "\n";

		return out.str();
	}

} // namespace trim
