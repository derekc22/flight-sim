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
    struct PropellerOmegaState_T {
        T omega = T(0.0);
        T omega_dot = T(0.0);
    };

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho);

    template <typename T>
    T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor, const T& omega, const atmospheric::AirDensity& rho);

    template <typename T>
    dynamics::Wrench_T<T> step_propulsor_forces_moments_T(const actuators::PropulsorActuator& propulsor, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const T& thrust, const T& omega_dot);

    template <typename T>
    dynamics::Wrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<T>& u, const PropulsorOmegaDot_T<T>& omega_dot);

    dynamics::Wrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<double>& u, double dt);

    void commit_propeller_omega_state(actuators::PropulsorActuator& propulsor, const PropellerOmegaState_T<double>& propeller_omega);

    void commit_propellers_omega_state(actuators::PropulsorActuators& propulsor_actuators, const PropellerOmegaState_T<double>& front_propulsor_omega, const PropellerOmegaState_T<double>& left_propulsor_omega, const PropellerOmegaState_T<double>& right_propulsor_omega);

    template <typename T>
    PropellerOmegaState_T<T> compute_propeller_omega_state_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, T dt);

    template <typename T>
    T compute_propeller_omega_dot_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, T dt);

    template <typename T>
    PropulsorOmegaDot_T<T> compute_propellers_omega_dot_T(const actuators::PropulsorActuators& propulsor_actuators, const actuators::PropulsorActuatorInputs_T<T>& u, const atmospheric::StaticAtmosphericState& atm, T dt);

}

#include "simulation/propulsion/public.tpp"
