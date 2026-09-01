#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion {

    template <typename T>
    T compute_propeller_omega_T(const actuators::PropulsorActuator& propulsor, const T& thrust, const atmospheric::AirDensity& rho);

    template <typename T>
    PropellerOmegaState_T<T> compute_propeller_omega_state_T(const actuators::PropulsorActuator& propulsor, T thrust, const atmospheric::AirDensity& rho, const std::optional<double>& previous_omega, T dt, bool steady_state);

    template <typename T>
    PropellerOmegaStateSet_T<T> compute_propeller_omega_state_set_T(const actuators::PropulsorActuators& propulsors, const PropulsionState& previous_state, const actuators::PropulsorActuatorInputs_T<T>& actuator_inputs, const atmospheric::AirDensity& rho, T dt, bool steady_state);

    PropellerOmegaStateSet_T<double> compute_propeller_omega_state_set(const actuators::PropulsorActuators& propulsors, const PropulsionState& previous_state, const actuators::PropulsorActuatorInputs_T<double>& actuator_inputs, const atmospheric::AirDensity& rho, double dt, bool steady_state);

}

#include "simulation/propulsion/public/detail/state.tpp"
