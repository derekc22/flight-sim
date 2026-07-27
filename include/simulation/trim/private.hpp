#pragma once
#include <cstddef>
#include <string>
#include <Eigen/Dense>
#include "simulation/trim/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    inline constexpr std::size_t trim_residual_dim = constants::state_input_dim;

    template <typename T>
    using TrimResidualVector_T = constants::MatrixX_T<T, trim_residual_dim, 1>;

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

    template <typename T>
    TrimResidual_T<T> pack_trim_residual_T(const TrimResidualVector_T<T>& residual);

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const operating::StateInputVector_T<T>& xu, autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions);

    TrimResidualVector_T<double> fetch_trim_residual_weights(const TrimSolveOptions& options);

    double residual_norm_inf(const TrimResidualVector_T<double>& residual);

    void validate_trim_solve_options(const TrimSolveOptions& options);

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations);

    TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu, autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions);

    TrimSolution solve_trim(const TrimProblem& problem, autodiff::AutoDiffModel& model, TrimSolveOptions options = {});

}

#include "simulation/trim/private.tpp"
