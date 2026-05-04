#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace trim {

    inline constexpr std::size_t trim_variable_dim = constants::state_dim + constants::input_dim;
    inline constexpr std::size_t trim_residual_dim = trim_variable_dim;

    template <typename T>
    using TrimVariablesVector_T = Eigen::Matrix<T, trim_variable_dim, 1>;

    template <typename T>
    using TrimActuatorInputsVector_T = Eigen::Matrix<T, constants::input_dim, 1>;

    template <typename T>
    using TrimStateVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    template <typename T>
    using TrimStateDotVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    template <typename T>
    using TrimResidualVector_T = Eigen::Matrix<T, trim_residual_dim, 1>;

    using TrimResidualJacobian = Eigen::Matrix<double, trim_residual_dim, trim_variable_dim>;

    struct TrimFixedActuatorInputs {
        double flap = 0.0;
        double spoiler = 0.0;
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
    struct TrimStateDot {
        T vx_dot = T(0);
        T vy_dot = T(0);
        T vz_dot = T(0);

        T p_dot = T(0);
        T q_dot = T(0);
        T r_dot = T(0);

        T phi_dot = T(0);
        T theta_dot = T(0);
    };

    template <typename T>
    struct TrimActuatorInputs {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };

    struct TrimTarget {
        double beta = 0.0;
        double phi = 0.0;
        double theta = 0.0;
        double vx = 0.0;
        double vz = 0.0;
        double psi_dot = 0.0;
    };

    struct TrimConditions {
        atmospheric::StaticAtmosphericState static_atmospheric_state;
        atmospheric::Wind windB{ constants::Zero3 };
    };

    struct TrimSolveOptions {
        std::size_t max_iterations = 30;
        double residual_tolerance = 1e-2;
        double step_tolerance = 1e-8;
        double initial_damping = 1e-6;
        double damping_growth = 10.0;
        double linear_accel_scale = constants::g_earth;
        double angular_accel_scale = util::deg_to_rad(30.0);
        double angle_rate_scale = util::deg_to_rad(10.0);
        double angle_err_scale = util::deg_to_rad(5.0);
        double vel_err_scale = 5.0;
        double backtrack_scale = 0.5;
        double min_step_scale = 1.0 / 1024.0;
    };

    template <typename T>
    struct TrimProblem {
        TrimTarget target;
        TrimConditions conditions;
        TrimState<T> state_guess;
        TrimActuatorInputs<T> input_guess;
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
        T beta_err = T(0);
        T phi_err = T(0);
        T theta_err = T(0);

        T vx_err = T(0);
        T vz_err = T(0);
        T psi_dot_err = T(0);
    };

    struct TrimSolution {
        TrimState<double> state;
        TrimActuatorInputs<double> input;
        TrimConditions conditions;
        dynamics::Wrench wrench{};
        TrimResidual<double> residual;
        TrimResidual<double> weighted_residual;
        TrimVariablesVector_T<double> variables = TrimVariablesVector_T<double>::Zero();
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        // double residual_norm_2 = 0.0;
        // double residual_norm_inf = 0.0;
        double weighted_residual_norm_2 = 0.0;
        double weighted_residual_norm_inf = 0.0;
    };
}
