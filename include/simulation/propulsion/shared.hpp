#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/dynamics/shared.hpp"

namespace propulsion {

    struct PropulsiveWrench : dynamics::Wrench {};

    template <typename T>
    struct PropulsorOmegaDot_T {
        T front_propulsor = T(0);
        T left_propulsor = T(0);
        T right_propulsor = T(0);
    };

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::PropulsorActuatorInputs_T<T>& u, const PropulsorOmegaDot_T<T>& omega_dot);

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::PropulsorActuatorInputs_T<double>& u);

}

#include "simulation/propulsion/shared.tpp"
