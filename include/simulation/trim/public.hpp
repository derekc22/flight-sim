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

namespace control { struct ControlOutput; } // forward declare
namespace vehicles { struct Aircraft; } // forward declare

namespace trim {

    template <typename T>
    struct TrimResidual_T {
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
        operating::OperatingPoint operating_point;
        operating::OperatingConditions conditions;
        dynamics::Wrench wrench{};
        TrimResidual_T<double> residual;
        TrimResidual_T<double> weighted_residual;
        operating::StateInputVector_T<double> variables = operating::StateInputVector_T<double>::Zero();
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        double weighted_residual_norm_2 = 0.0;
        double weighted_residual_norm_inf = 0.0;
    };

    template <typename T>
    T get_control_from_solver_space_T(const T& u_solver, double limit_min, double limit_max);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const operating::StateInputVector_T<T>& z, const actuators::ActuatorLimits_T<double>& actuator_limits);

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const atmospheric::Wind& wind);

    std::string print_trim_solution(const TrimSolution& trim_sol);

    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_trim(const dynamics::RigidBodyState& Xt, const TrimSolution& trim_sol);

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol);

    /** @deprecated */
    // void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol);

    std::pair<actuators::SurfaceActuatorInputs_T<double>, actuators::PropulsorActuatorInputs_T<double>> update_actuators_from_trim(actuators::SurfaceActuatorInputs_T<double>& surface_actuator_inputs, actuators::PropulsorActuatorInputs_T<double>& propulsor_actuator_inputs, const TrimSolution& trim_sol);

}

#include "simulation/trim/public.tpp"
