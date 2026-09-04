#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/operating/public/data/types.hpp"

#include <cstddef>

namespace trim
{

	inline constexpr std::size_t trim_residual_dim = constants::state_input_dim;

	template <typename T> using TrimResidualVector_T = constants::MatrixX_T<T, trim_residual_dim, 1>;

	using TrimResidualJacobian = constants::MatrixX_T<double, trim_residual_dim, constants::state_input_dim>;

	struct TrimTarget {
		double beta = 0.0;
		double phi = 0.0;
		double theta = 0.0;
		double vx = 0.0;
		double vz = 0.0;
		double psi_dot = 0.0;
	};

	struct TrimSolveOptions {
		std::size_t max_iterations = 30;
		double residual_tolerance = 1e-2;
		double step_tolerance = 1e-8;
		double initial_damping = 1e-6;
		double damping_growth = 10.0;
		double linear_accel_scale = constants::g_earth;
		double angular_accel_scale = constants::pi / 6.0;
		double angle_rate_scale = constants::pi / 18.0;
		double angle_err_scale = constants::pi / 36.0;
		double vel_err_scale = 5.0;
		double backtrack_scale = 0.5;
		double min_step_scale = 1.0 / 1024.0;
	};

	struct TrimProblem {
		TrimTarget target;
		operating::OperatingConditions conditions;
		operating::OperatingPoint_T<double> initial_guess;
	};

} // namespace trim
