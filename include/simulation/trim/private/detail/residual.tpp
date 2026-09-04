#pragma once
#include "simulation/aerodynamics/public/detail/state.hpp"
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/operating/public/data/helpers.hpp"
#include "simulation/trim/private/detail/residual.hpp"

namespace trim
{

	template <typename T>
	TrimResidualVector_T<T> compute_trim_residual_vector_T(
		const operating::StateInputVector_T<T>& xu,
		const autodiff::AutoDiffModel& model,
		const TrimTarget& target,
		const operating::OperatingConditions& conditions)
	{
		const operating::OperatingPoint_T<T> operating_point = operating::pack_state_input_T(xu);
		const dynamics::State_T<T> x = operating_point.state;

		const dynamics::StateDot_T<T> x_dot =
			autodiff::compute_state_dot_T<T>(operating_point, model, conditions, constants::dt);
		const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
		const aerodynamics::AerodynamicState_T<T> aero =
			aerodynamics::compute_aerodynamic_state_T<T>(twist, conditions.windB);
		const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

		TrimResidualVector_T<T> residual;
		residual << x_dot.vx_dot, x_dot.vy_dot, x_dot.vz_dot, x_dot.p_dot, x_dot.q_dot, x_dot.r_dot, x_dot.phi_dot,
			x_dot.theta_dot, aero.beta - T(target.beta), x.phi - T(target.phi), x.theta - T(target.theta),
			x.vx - T(target.vx), x.vz - T(target.vz), eul_dot.z() - T(target.psi_dot);
		return residual;
	}
} // namespace trim
