#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include <utility>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/operating/public.hpp"

namespace control { struct ControlOutput; }
namespace vehicles { struct Aircraft; }

namespace trim {

    inline constexpr std::size_t trim_variable_dim = constants::state_dim + constants::input_dim;

    template <typename T>
    using TrimVariablesVector_T = Eigen::Matrix<T, trim_variable_dim, 1>;

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

    struct TrimModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::PropulsorActuators& propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits;
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };

    struct TrimSolution {
        operating::OperatingPoint operating_point;
        operating::OperatingConditions conditions;
        dynamics::Wrench wrench{};
        TrimResidual<double> residual;
        TrimResidual<double> weighted_residual;
        TrimVariablesVector_T<double> variables = TrimVariablesVector_T<double>::Zero();
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        double weighted_residual_norm_2 = 0.0;
        double weighted_residual_norm_inf = 0.0;
    };

    template <typename T>
    T get_control_from_solver_space_T(const T& u_solver, double limit_min, double limit_max);

    template <typename T>
    dynamics::Twist_T<T> build_twist_from_trim_state_T(const dynamics::State_T<T>& x);

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> build_surface_actuator_inputs_from_trim_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs);

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> build_propulsor_actuator_inputs_from_trim_T(const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    constants::Vector3_T<T> gB_T(const T& phi, const T& theta);

    template <typename T>
    dynamics::Wrench_T<T> compute_trim_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const operating::OperatingConditions& conditions);

    template <typename T>
    dynamics::StateDot_T<T> compute_trim_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const operating::OperatingConditions& conditions);

    template <typename T>
    TrimVariablesVector_T<T> unpack_trim_variables_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::State_T<T> pack_trim_state_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z, const actuators::ActuatorLimits_T<double>& actuator_limits);

    TrimModel build_trim_model(vehicles::Aircraft& aircraft);

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind);

    std::string print_trim_solution(const TrimSolution& trim_sol);

    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& xN_t, const TrimSolution& trim_sol);

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol);

    /** @deprecated */
    // void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol);

    std::pair<actuators::SurfaceActuatorInputs_T<double>, actuators::PropulsorActuatorInputs_T<double>> update_actuators_from_trim(actuators::SurfaceActuatorInputs_T<double>& surface_actuator_inputs, actuators::PropulsorActuatorInputs_T<double>& propulsor_actuator_inputs, const TrimSolution& trim_sol);

}

#include "simulation/trim/public.tpp"
