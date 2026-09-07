#pragma once
#include "simulation/aerodynamics/public/components/surface.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/cppad.hpp"
#include "simulation/util/public/linalg.hpp"
#include "simulation/util/public/math.hpp"
#include "simulation/util/public/trig.hpp"

namespace aerodynamics
{
	template <typename T>
	SurfaceKinematics_T<T> Surface::compute_kinematics(
		const SurfaceInput_T<T>& input)
	{
		SurfaceKinematics_T<T> out;
		const constants::Vector3_T<T> pG_acG = pB_acB.cast<T>() - input.pB_GB;
		out.vB_rel = (input.twist.v - input.windB.data.cast<T>()) + input.twist.w.cross(pG_acG);
		out.V = out.vB_rel.norm();

		if (out.V < T(constants::eps)) {
			return out;
		}

		const constants::Vector3_T<T> n_hat = n.cast<T>();
		const T arg = util::clamp_to_1(out.vB_rel.dot(n_hat) / out.V);

		out.alpha = T(i) - util::asin(arg);
		out.qbar = T(0.5 * input.atm.rho.data) * out.V * out.V;
		out.p_hat = input.twist.w.x() * T(span / 2.0) / out.V;
		out.q_hat = input.twist.w.y() * T(chord / 2.0) / out.V;
		out.r_hat = input.twist.w.z() * T(span / 2.0) / out.V;
		return out;
	}

	template <typename T>
	SurfaceCoefficients_T<T> Surface::compute_coefficients(
		const SurfaceKinematics_T<T>& sk,
		const actuators::SurfaceActuatorInputs_T<T>& u)
	{
		const double CLalpha = 2.0 * constants::pi * (AR / (2.0 + AR));

		SurfaceCoefficients_T<T> out;
		out.CL = T(CL0) + T(CLalpha) * sk.alpha;
		out.CM = T(CM0) + T(CMa) * sk.alpha;

		out.CL += T(dyn.p_hat.dCL) * sk.p_hat + T(dyn.q_hat.dCL) * sk.q_hat + T(dyn.r_hat.dCL) * sk.r_hat;
		out.CM += T(dyn.p_hat.dCM) * sk.p_hat + T(dyn.q_hat.dCM) * sk.q_hat + T(dyn.r_hat.dCM) * sk.r_hat;
		out.CD += T(dyn.p_hat.dCD) * sk.p_hat + T(dyn.q_hat.dCD) * sk.q_hat + T(dyn.r_hat.dCD) * sk.r_hat;

		for (SurfaceEffector& effector : effectors) {
			const SurfaceCoefficients_T<T> contribution = effector.step<T>(u);
			out.CL += contribution.CL;
			out.CD += contribution.CD;
			out.CM += contribution.CM;
		}

		out.CD +=
			T(CD0) + T(CDa) * (sk.alpha - T(a0)) * (sk.alpha - T(a0)) + (out.CL * out.CL) / T(constants::pi * e * AR);
		return out;
	}

	template <typename T>
	dynamics::Wrench_T<T> Surface::compute_loads(
		const constants::Vector3_T<T>& pB_GB,
		const SurfaceKinematics_T<T>& sk,
		const SurfaceCoefficients_T<T>& sc)
	{
		dynamics::Wrench_T<T> out;
		if (sk.V < T(constants::eps)) {
			return out;
		}

		const constants::Vector3_T<T> n_hat = n.cast<T>();
		const constants::Vector3_T<T> d_hat = -sk.vB_rel / sk.V;

		const constants::Vector3_T<T> lift_axis = n_hat - n_hat.dot(d_hat) * d_hat;
		const constants::Vector3_T<T> l_hat = util::norm(lift_axis);

		const constants::Vector3_T<T> moment_axis = l_hat.cross(d_hat);
		const constants::Vector3_T<T> m_hat = util::norm(moment_axis);

		const T L = sk.qbar * T(area) * sc.CL;
		const T D = sk.qbar * T(area) * sc.CD;
		const T Mmag = sk.qbar * T(area * chord) * sc.CM;

		out.F = L * l_hat + D * d_hat;
		const constants::Vector3_T<T> pG_acG = pB_acB.cast<T>() - pB_GB;
		out.M = pG_acG.cross(out.F) + Mmag * m_hat;
		return out;
	}

	template <typename T>
	dynamics::Wrench_T<T> Surface::step(
		const SurfaceInput_T<T>& input)
	{
		const SurfaceKinematics_T<T> sk = compute_kinematics<T>(input);
		const SurfaceCoefficients_T<T> sc = compute_coefficients<T>(sk, input.u);
		return compute_loads<T>(input.pB_GB, sk, sc);
	}

} // namespace aerodynamics
