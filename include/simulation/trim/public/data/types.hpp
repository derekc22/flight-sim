#pragma once
#include <cstddef>
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace trim {

	template <typename T> struct TrimResidual_T {
		T vx_dot = T(0.0);
		T vy_dot = T(0.0);
		T vz_dot = T(0.0);
		T p_dot = T(0.0);
		T q_dot = T(0.0);
		T r_dot = T(0.0);
		T phi_dot = T(0.0);
		T theta_dot = T(0.0);
		T beta_err = T(0.0);
		T phi_err = T(0.0);
		T theta_err = T(0.0);
		T vx_err = T(0.0);
		T vz_err = T(0.0);
		T psi_dot_err = T(0.0);
	};

	struct TrimSolution {
		operating::OperatingPoint_T<double> operating_point;
		operating::OperatingConditions conditions;
		dynamics::Wrench_T<double> wrench{};
		TrimResidual_T<double> residual;
		TrimResidual_T<double> weighted_residual;
		bool attempted = false;
		bool converged = false;
		std::size_t iterations = 0;
		double weighted_residual_norm_2 = 0.0;
		double weighted_residual_norm_inf = 0.0;
	};

} // namespace trim
