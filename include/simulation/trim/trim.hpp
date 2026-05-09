#pragma once
#include <cstddef>
#include <utility> // For std::pair
#include <Eigen/Dense>
#include "simulation/trim/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/propulsion/propulsion.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace trim {

    inline constexpr std::size_t trim_residual_dim = trim_variable_dim;

    template <typename T>
    using TrimResidualVector_T = Eigen::Matrix<T, trim_residual_dim, 1>;

    using TrimResidualJacobian = Eigen::Matrix<double, trim_residual_dim, trim_variable_dim>;

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
        TrimConditions conditions;
        dynamics::State_T<T> state_guess;
        actuators::ActuatorInputs_T<T> input_guess;
    };

    template <typename T>
    dynamics::StateDot_T<T> compute_trim_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimConditions& conditions);

    template <typename T>
    TrimResidual<T> compute_trim_residual(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions);

    template <typename T>
    TrimVariablesVector_T<T> unpack_trim_variables_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::State_T<T> pack_trim_state_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    TrimResidualVector_T<T> unpack_trim_residual_T(const TrimResidual<T>& residual);

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariablesVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimResidualVector_T<double> compute_trim_residual_vector(const TrimVariablesVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimResidualJacobian compute_trim_residual_jac(const TrimVariablesVector_T<double>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls);

    TrimSolution solve_trim(const TrimProblem<double>& problem, const TrimModel& model, TrimSolveOptions options = {});

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind);

    std::string print_trim_solution(const TrimSolution& trim_sol);

    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& xN_t, const TrimSolution& trim_sol);

    /** @deprecated */
    // void update_actuators_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol);

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol);

    dynamics::StateVector_T<double> unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t);
}

#include "simulation/trim/trim.tpp"
