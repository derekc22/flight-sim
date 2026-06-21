#pragma once
#include <cstddef>
#include <utility> // For std::pair
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
    using TrimResidualVector_T = Eigen::Matrix<T, trim_residual_dim, 1>;

    using TrimResidualJacobian = Eigen::Matrix<double, trim_residual_dim, constants::state_input_dim>;

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

    template <typename T>
    struct TrimProblem {
        TrimTarget target;
        operating::OperatingConditions conditions;
        dynamics::State_T<T> state_guess;
        actuators::ActuatorInputs_T<T> input_guess;
    };

    template <typename T>
    TrimResidual_T<T> compute_trim_residual_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions);

    template <typename T>
    TrimResidualVector_T<T> unpack_trim_residual_T(const TrimResidual_T<T>& residual);

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const operating::StateInputVector_T<T>& xu, const autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions, bool use_physical_controls);

    double send_control_to_solver_space(double u, double limit_min, double limit_max);

    operating::StateInputVector_T<double> unpack_trim_solver_variables(const dynamics::State_T<double>& x, const actuators::ActuatorInputs_T<double>& u, const actuators::ActuatorLimits_T<double>& actuator_limits);

    TrimResidualVector_T<double> fetch_trim_residual_weights(const TrimSolveOptions& options);

    double residual_norm_inf(const TrimResidualVector_T<double>& residual);

    void validate_trim_solve_options(const TrimSolveOptions& options);

    dynamics::Wrench compute_trim_wrench(const dynamics::State_T<double>& x, const actuators::ActuatorInputs_T<double>& u, const autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions);

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, const autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations);

    TrimResidualVector_T<double> compute_trim_residual_vector(const operating::StateInputVector_T<double>& xu, const autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions, bool use_physical_controls);

    TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu, const autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions, bool use_physical_controls);

    TrimSolution solve_trim(const TrimProblem<double>& problem, const autodiff::AutoDiffModel& model, TrimSolveOptions options = {});

}

#include "simulation/trim/private.tpp"
