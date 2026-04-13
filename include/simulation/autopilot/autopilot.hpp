#pragma once

#include <cstddef>
#include <utility> // For std::pair
#include <Eigen/Dense>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/actuators/actuators.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace autopilot { // to encompass autonomy and trim

    inline constexpr std::size_t trim_state_dofs = 8;
    inline constexpr std::size_t trim_input_dofs = 3;
    inline constexpr std::size_t trim_variable_dofs = trim_state_dofs + trim_input_dofs;
    inline constexpr std::size_t trim_residual_dofs = 11;

    template <typename T>
    using TrimVariableVector_T = Eigen::Matrix<T, trim_variable_dofs, 1>;

    template <typename T>
    using TrimControlSurfaceInputsVector_T = Eigen::Matrix<T, trim_input_dofs, 1>;

    template <typename T>
    using TrimStateVector_T = Eigen::Matrix<T, trim_state_dofs, 1>;

    template <typename T>
    using TrimStateDotVector_T = Eigen::Matrix<T, trim_state_dofs, 1>;

    template <typename T>
    using TrimResidualVector_T = Eigen::Matrix<T, trim_residual_dofs, 1>;

    using TrimResidualJacobian = Eigen::Matrix<double, trim_residual_dofs, trim_variable_dofs>;

    struct TrimFixedControls {
        double flaps = 0.0;
        double spoilers = 0.0;
    };

    struct TrimModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::ActuatorProperties& actuator;
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
    struct TrimControlSurfaceInputs {
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
        atmospheric::StaticAtmosphericState static_atmospheric_state;
        atmospheric::Wind windB{ constants::Zero3 };
    };

    struct TrimSolveOptions {
        std::size_t max_iterations = 30;
        double residual_tolerance = 1e-6;
        double step_tolerance = 1e-8;
        double initial_damping = 1e-6;
        double damping_growth = 10.0;
        double linear_accel_scale = constants::g_earth;
        double angular_accel_scale = util::deg_to_rad(30.0);
        double angle_rate_scale = util::deg_to_rad(10.0);
        double angle_error_scale = util::deg_to_rad(5.0);
        double backtrack_scale = 0.5;
        double min_step_scale = 1.0 / 1024.0;
    };

    template <typename T>
    struct TrimProblem {
        TrimTarget target;
        TrimConditions conditions;
        TrimState<T> state_guess;
        TrimControlSurfaceInputs<T> input_guess;
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
        TrimControlSurfaceInputs<double> input;
        TrimConditions conditions;
        dynamics::Wrench wrench{};
        TrimResidual<double> residual;
        TrimVariableVector_T<double> variables = TrimVariableVector_T<double>::Zero();
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        double residual_norm_2 = 0.0;
        double residual_norm_inf = 0.0;
    };

    template <typename T>
    TrimStateDot<T> compute_trim_state_dot_T(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u, const TrimModel& model, const TrimConditions& conditions);

    template <typename T>
    TrimResidual<T> compute_trim_residual(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions);

    template <typename T>
    TrimVariableVector_T<T> pack_trim_variables_T(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u);

    template <typename T>
    TrimControlSurfaceInputsVector_T<T> pack_trim_control_surface_inputs_T(const TrimControlSurfaceInputs<T>& u);

    template <typename T>
    TrimStateVector_T<T> pack_trim_state_T(const TrimState<T>& x);

    template <typename T>
    TrimStateDotVector_T<T> pack_trim_state_dot_T(const TrimStateDot<T>& x_dot);

    template <typename T>
    TrimState<T> unpack_trim_state_T(const TrimVariableVector_T<T>& z);

    template <typename T>
    TrimControlSurfaceInputs<T> unpack_trim_input_T(const TrimVariableVector_T<T>& z);

    template <typename T>
    TrimResidualVector_T<T> pack_trim_residual_T(const TrimResidual<T>& residual);

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariableVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimResidualVector_T<double> compute_trim_residual_vector(const TrimVariableVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimResidualJacobian compute_trim_residual_jac(const TrimVariableVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModel& model, TrimSolveOptions options = {});

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind);

    std::string print_trim_solution(const TrimSolution& trim_sol);

    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& xN_t, const TrimSolution& trim_sol);

    void update_actuators_from_trim(actuators::Actuators& actuators,  const TrimSolution& trim_sol);
}

#include "simulation/autopilot/autopilot.tpp"
