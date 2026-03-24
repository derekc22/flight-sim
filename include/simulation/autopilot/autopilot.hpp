#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/control/control.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/vehicles/vehicles.hpp"

namespace autopilot { // to encompass autonomy and trim

    inline constexpr std::size_t trim_state_dofs = 8;
    inline constexpr std::size_t trim_input_dofs = 3;
    inline constexpr std::size_t trim_variable_dofs = trim_state_dofs + trim_input_dofs;
    inline constexpr std::size_t trim_residual_dofs = 11;

    struct TrimFixedControls {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    struct TrimModelContext {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const control::ControlProperties& control;
        TrimFixedControls fixed_controls{};
    };

    template <typename T>
    struct TrimState {
        T vx = T(0);
        T vy = T(0);
        T vz = T(0);

        T p = T(0);
        T q = T(0);
        T r = T(0);

        T phi = T(0);
        T theta = T(0);
    };

    template <typename T>
    struct TrimInput {
        T elevator = T(0);
        T aileron = T(0);
        T rudder = T(0);
    };

    struct TrimTarget {
        double beta = 0.0;
        double phi = 0.0;
        double theta = 0.0;
    };

    struct TrimConditions {
        atmospheric::Density rho{ 0.0 };
        atmospheric::Wind windB{ global::Zero3 };
    };

    struct TrimSolveOptions {
        std::size_t max_iterations = 30;
        double residual_tolerance = 1e-6;
        double step_tolerance = 1e-8;
        double initial_damping = 1e-6;
        double damping_growth = 10.0;
        double linear_accel_scale = global::gravity;
        double angular_accel_scale = global::deg_to_rad(30.0);
        double angle_rate_scale = global::deg_to_rad(10.0);
        double angle_error_scale = global::deg_to_rad(5.0);
        double backtrack_scale = 0.5;
        double min_step_scale = 1.0 / 1024.0;
    };

    template <typename T>
    struct TrimProblem {
        TrimTarget target;
        TrimConditions conditions;
        TrimState<T> state_guess;
        TrimInput<T> input_guess;
    };

    template <typename T>
    struct TrimResidual {
        T vx_dot = T(0);
        T vy_dot = T(0);
        T vz_dot = T(0);

        T p_dot = T(0);
        T q_dot = T(0);
        T r_dot = T(0);

        T phi_dot = T(0);
        T theta_dot = T(0);
        T beta_error = T(0);
        T phi_error = T(0);
        T theta_error = T(0);
    };

    struct TrimSolution {
        TrimState<double> state;
        TrimInput<double> input;
        TrimResidual<double> residual;
        std::vector<double> variables;
        bool converged = false;
        std::size_t iterations = 0;
        double residual_norm_2 = 0.0;
        double residual_norm_inf = 0.0;
    };

    template <typename T>
    TrimResidual<T> evaluate_trim_residual(const TrimState<T>& x, const TrimInput<T>& u, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions);

    template <typename T>
    std::vector<T> pack_trim_variables_T(const TrimState<T>& x, const TrimInput<T>& u);

    template <typename T>
    TrimState<T> unpack_trim_state_T(const std::vector<T>& z);

    template <typename T>
    TrimInput<T> unpack_trim_input_T(const std::vector<T>& z);

    template <typename T>
    std::vector<T> pack_trim_residual_T(const TrimResidual<T>& residual);

    template <typename T>
    std::vector<T> evaluate_trim_residual_vector_T(const std::vector<T>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions);

    std::vector<double> evaluate_trim_residual_vector(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions);

    std::vector<double> compute_trim_residual_jac(const std::vector<double>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions);

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModelContext& model, TrimSolveOptions options = {});

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind);
}

#include "simulation/autopilot/autopilot.tpp"
