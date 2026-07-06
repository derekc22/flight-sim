#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace propulsion {

    template <typename T>
    struct PropulsorOmegaDot_T {
        T front_propulsor = T(0.0);
        T left_propulsor = T(0.0);
        T right_propulsor = T(0.0);
    };

    template <typename T>
    T compute_propeller_omega_T(
        const actuators::PropulsorActuator& propulsor, 
        const T& thrust, 
        const atmospheric::AirDensity& rho
    );

    template <typename T>
    T compute_propeller_torque_T(
        const actuators::PropulsorActuator& propulsor, 
        const T& omega, 
        const atmospheric::AirDensity& rho
    );

    template <typename T>
    dynamics::Wrench_T<T> step_propulsor_forces_moments_T(
        const actuators::PropulsorActuator& propulsor, 
        const dynamics::Twist_T<T>& twist, 
        const atmospheric::StaticAtmosphericState& atm, 
        const T& thrust, 
        const T& omega_dot
    );

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(
        actuators::PropulsorActuators& propulsor_actuators, 
        const dynamics::Twist_T<T>& twist, 
        const atmospheric::StaticAtmosphericState& atm, 
        const actuators::PropulsorActuatorInputs_T<T>& u, 
        T dt,
        bool steady_state
    );

    dynamics::Wrench step_propulsive_forces_moments(
        actuators::PropulsorActuators& propulsor_actuators, 
        const dynamics::RigidBodyState& X, 
        const atmospheric::StaticAtmosphericState& atm, 
        const actuators::PropulsorActuatorInputs_T<double>& u, 
        double dt
    );

    template <typename T>
    T step_propeller_omega_dot_T(
        actuators::PropulsorActuator& propulsor, 
        T thrust, 
        const atmospheric::AirDensity& rho, 
        T dt
    );

    template <typename T>
    PropulsorOmegaDot_T<T> step_propellers_omega_dot_T(
        actuators::PropulsorActuators& propulsor_actuators, 
        const actuators::PropulsorActuatorInputs_T<T>& u, 
        const atmospheric::StaticAtmosphericState& atm, 
        T dt
    );

}

#include "simulation/propulsion/public.tpp"
