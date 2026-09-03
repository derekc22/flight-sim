#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/util/public/cppad.hpp"
#include "simulation/util/public/linalg.hpp"
#include "simulation/util/public/math.hpp"
#include "simulation/util/public/trig.hpp"

namespace aerodynamics {

	template <typename T>
	SurfaceKinematics_T<T> compute_surface_kinematics_T(
	    const Surface& s,
	    const constants::Vector3_T<T>& pB_GB,
	    const dynamics::Twist_T<T>& twist,
	    const atmospheric::StaticAtmosphericState& atm,
	    const atmospheric::Wind& windB)
	{
		SurfaceKinematics_T<T> out;
		const constants::Vector3_T<T> pG_acG = s.pB_acB.cast<T>() - pB_GB;
		out.vB_rel = (twist.v - windB.data.cast<T>()) + twist.w.cross(pG_acG);
		out.V = out.vB_rel.norm();

		if (out.V < T(constants::eps)) {
			return out;
		}

		const constants::Vector3_T<T> n_hat = s.n.cast<T>();
		const T arg = util::clamp_to_1(out.vB_rel.dot(n_hat) / out.V);

		out.alpha = T(s.i) - util::asin(arg);
		out.qbar = T(0.5 * atm.rho.data) * out.V * out.V;
		out.p_hat = twist.w.x() * T(s.span / 2.0) / out.V;
		out.q_hat = twist.w.y() * T(s.chord / 2.0) / out.V;
		out.r_hat = twist.w.z() * T(s.span / 2.0) / out.V;
		return out;
	}

	template <typename T>
	SurfaceCoefficients_T<T> compute_surface_coefficients_T(
	    const Surface& s,
	    const SurfaceKinematics_T<T>& sk,
	    const actuators::SurfaceActuatorInputs_T<T>& u)
	{
		const DynamicDerivatives& dyn = s.dyn;
		const ControlDerivatives& ctrl = s.ctrl;
		const double CLalpha = 2.0 * constants::pi * (s.AR / (2.0 + s.AR));

		SurfaceCoefficients_T<T> out;
		out.CL = T(s.CL0) + T(CLalpha) * sk.alpha;
		out.CM = T(s.CM0) + T(s.CMa) * sk.alpha;

		const T elevator_cmd_abs = util::smooth_abs(u.elevator_cmd);
		const T aileron_cmd_abs = util::smooth_abs(u.aileron_cmd);
		const T rudder_cmd_abs = util::smooth_abs(u.rudder_cmd);
		const T flap_cmd_abs = util::smooth_abs(u.flap_cmd);
		const T spoiler_cmd_abs = util::smooth_abs(u.spoiler_cmd);

		out.CL += T(dyn.CL_phat) * sk.p_hat + T(dyn.CL_qhat) * sk.q_hat + T(dyn.CL_rhat) * sk.r_hat;
		out.CM += T(dyn.CM_phat) * sk.p_hat + T(dyn.CM_qhat) * sk.q_hat + T(dyn.CM_rhat) * sk.r_hat;
		out.CD += T(dyn.CD_phat) * sk.p_hat + T(dyn.CD_qhat) * sk.q_hat + T(dyn.CD_rhat) * sk.r_hat;

		out.CL += T(ctrl.dCL_de) * u.elevator_cmd + T(ctrl.dCL_da) * u.aileron_cmd + T(ctrl.dCL_dr) * u.rudder_cmd +
		    T(ctrl.dCL_df) * u.flap_cmd + T(ctrl.dCL_ds) * u.spoiler_cmd;
		out.CM += T(ctrl.dCM_de) * u.elevator_cmd + T(ctrl.dCM_da) * u.aileron_cmd + T(ctrl.dCM_dr) * u.rudder_cmd +
		    T(ctrl.dCM_df) * u.flap_cmd + T(ctrl.dCM_ds) * u.spoiler_cmd;
		out.CD += T(ctrl.dCD_de) * elevator_cmd_abs + T(ctrl.dCD_da) * aileron_cmd_abs +
		    T(ctrl.dCD_dr) * rudder_cmd_abs + T(ctrl.dCD_df) * flap_cmd_abs + T(ctrl.dCD_ds) * spoiler_cmd_abs;

		out.CD += T(s.CD0) + T(s.CDa) * (sk.alpha - T(s.a0)) * (sk.alpha - T(s.a0)) +
		    (out.CL * out.CL) / T(constants::pi * s.e * s.AR);
		return out;
	}

	template <typename T>
	dynamics::Wrench_T<T> compute_surface_loads_T(
	    const Surface& s,
	    const constants::Vector3_T<T>& pB_GB,
	    const SurfaceKinematics_T<T>& sk,
	    const SurfaceCoefficients_T<T>& sc)
	{
		dynamics::Wrench_T<T> out;
		if (sk.V < T(constants::eps)) {
			return out;
		}

		const constants::Vector3_T<T> n_hat = s.n.cast<T>();
		const constants::Vector3_T<T> d_hat = -sk.vB_rel / sk.V;

		const constants::Vector3_T<T> lift_axis = n_hat - n_hat.dot(d_hat) * d_hat;
		const constants::Vector3_T<T> l_hat = util::norm(lift_axis);

		const constants::Vector3_T<T> moment_axis = l_hat.cross(d_hat);
		const constants::Vector3_T<T> m_hat = util::norm(moment_axis);

		const T L = sk.qbar * T(s.area) * sc.CL;
		const T D = sk.qbar * T(s.area) * sc.CD;
		const T Mmag = sk.qbar * T(s.area * s.chord) * sc.CM;

		out.F = L * l_hat + D * d_hat;
		const constants::Vector3_T<T> pG_acG = s.pB_acB.cast<T>() - pB_GB;
		out.M = pG_acG.cross(out.F) + Mmag * m_hat;
		return out;
	}

	template <typename T>
	dynamics::Wrench_T<T> step_aero_forces_moments_T(
	    const std::vector<Surface>& surfaces,
	    const constants::Vector3_T<T>& pB_GB,
	    const dynamics::Twist_T<T>& twist,
	    const atmospheric::StaticAtmosphericState& atm,
	    const actuators::SurfaceActuatorInputs_T<T>& u,
	    const atmospheric::Wind& windB)
	{
		dynamics::Wrench_T<T> total;
		for (const Surface& s : surfaces) {
			const SurfaceKinematics_T<T> sk = compute_surface_kinematics_T<T>(s, pB_GB, twist, atm, windB);
			const SurfaceCoefficients_T<T> sc = compute_surface_coefficients_T<T>(s, sk, u);
			const dynamics::Wrench_T<T> loads = compute_surface_loads_T<T>(s, pB_GB, sk, sc);
			total.F += loads.F;
			total.M += loads.M;
		}
		return total;
	}

} // namespace aerodynamics
