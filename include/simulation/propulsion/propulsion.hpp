#pragma once
#include <optional>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/util/util.hpp"


namespace propulsion {

    struct PropulsiveWrench : dynamics::Wrench {};

   template <typename T>
    struct PropulsorActuatorInputs_T {
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };

   template <typename T>
    struct PropulsorOmegaDot_T {
        T front_propulsor = T(0);
        T left_propulsor = T(0);
        T right_propulsor = T(0);
    };

    template <typename T>
    struct PropulsiveWrench_T {
        constants::Vector3_T<T> F = constants::Zero3_T<T>;
        constants::Vector3_T<T> M = constants::Zero3_T<T>;
    };

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho);

    template <typename T>
    PropulsiveWrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const PropulsorActuatorInputs_T<T>& u, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const PropulsorOmegaDot_T<T>& omega_dot);

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const control::PropulsorActuatorInputs& u, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atmospheric_state);

}

#include "simulation/propulsion/propulsion.tpp"
