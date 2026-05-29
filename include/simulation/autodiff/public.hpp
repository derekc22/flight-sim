#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace autodiff {

    template <typename T>
    using AutoDiffVariableVector_T = Eigen::Matrix<T, constants::state_input_dim, 1>;

    struct AutoDiffModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::PropulsorActuators& propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits;
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };

    struct OperatingConditions {
        atmospheric::StaticAtmosphericState static_atm_state;
        atmospheric::Wind windB{ constants::Zero3 };
    };

    struct OperatingPoint {
        dynamics::State_T<double> state;
        actuators::ActuatorInputs_T<double> input;
    };

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft);

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> build_surface_actuator_inputs_from_autodiff_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs);

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> build_propulsor_actuator_inputs_from_autodiff_T(const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    constants::Vector3_T<T> gB_T(const T& phi, const T& theta);

    template <typename T>
    dynamics::Wrench_T<T> compute_autodiff_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const autodiff::OperatingConditions& conditions);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const autodiff::OperatingConditions& conditions);

    template <typename T>
    AutoDiffVariableVector_T<T> unpack_autodiff_variables_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::State_T<T> pack_autodiff_state_T(const AutoDiffVariableVector_T<T>& z);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_autodiff_actuator_inputs_T(const AutoDiffVariableVector_T<T>& z);

}

#include "simulation/autodiff/public.tpp"
