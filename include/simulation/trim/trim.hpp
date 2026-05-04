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
#include "simulation/propulsion/propulsion.hpp"
#include "simulation/trim/types.hpp"

namespace vehicles { struct Aircraft; } // forward declare
namespace control { struct ControlOutput; } // forward declare

namespace trim {

    struct TrimModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        TrimFixedActuatorInputs fixed_controls{};
    };

    template <typename T>
    TrimStateDot<T> compute_trim_state_dot_T(const TrimState<T>& x, const TrimActuatorInputs<T>& u, const TrimModel& model, const TrimConditions& conditions);

    template <typename T>
    TrimResidual<T> compute_trim_residual(const TrimState<T>& x, const TrimActuatorInputs<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions);

    template <typename T>
    TrimVariablesVector_T<T> unpack_trim_variables_T(const TrimState<T>& x, const TrimActuatorInputs<T>& u);

    template <typename T>
    TrimActuatorInputsVector_T<T> unpack_trim_actuator_inputs_T(const TrimActuatorInputs<T>& u);

    template <typename T>
    TrimStateVector_T<T> unpack_trim_state_T(const TrimState<T>& x);

    template <typename T>
    TrimStateDotVector_T<T> unpack_trim_state_dot_T(const TrimStateDot<T>& x_dot);

    template <typename T>
    TrimState<T> pack_trim_state_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    TrimActuatorInputs<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z);

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

    /** @deprecated DO NOT REFERENCE */
    // void update_actuators_from_trim(actuators::SurfaceActuators& surface_actuators,  actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol);

    control::ControlOutput set_control_inputs_from_trim(const TrimSolution& trim_sol);

    TrimStateVector_T<double> unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t);
}

#include "simulation/trim/trim.tpp"
