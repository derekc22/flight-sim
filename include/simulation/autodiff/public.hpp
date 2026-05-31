#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/operating/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace autodiff {

    struct AutoDiffModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::PropulsorActuators& propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits;
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft);

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> pack_surface_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs);

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> pack_propulsor_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const operating::OperatingConditions& conditions);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const operating::OperatingConditions& conditions);

}

#include "simulation/autodiff/public.tpp"
