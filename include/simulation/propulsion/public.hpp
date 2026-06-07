#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace propulsion {

    struct PropulsiveWrench : dynamics::Wrench {};

    template <typename T>
    struct PropulsorOmegaDot_T {
        T front_propulsor = T(0);
        T left_propulsor = T(0);
        T right_propulsor = T(0);
    };

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho);

    template <typename T>
    T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor, const T& omega, const atmospheric::AirDensity& rho);

    template <typename T>
    dynamics::Wrench_T<T> step_propulsor_forces_moments_T(const actuators::PropulsorActuator& propulsor, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm, const T& thrust, const T& omega_dot);

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm, const actuators::PropulsorActuatorInputs_T<T>& u, const PropulsorOmegaDot_T<T>& omega_dot);

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& static_atm, const actuators::PropulsorActuatorInputs_T<double>& u);

}

#include "simulation/propulsion/public.tpp"
