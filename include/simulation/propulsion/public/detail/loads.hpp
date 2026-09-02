#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion {

    template <typename T>
    T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor, const T& omega, const atmospheric::AirDensity& rho);

    template <typename T>
    dynamics::Wrench_T<T> compute_propulsor_loads_T(const actuators::PropulsorActuator& propulsor, const constants::Vector3_T<T>& center_of_gravity, const dynamics::Twist_T<T>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const T& thrust, const PropellerOmegaState_T<T>& propeller_state);

    template <typename T>
    dynamics::Wrench_T<T> compute_propulsive_loads_T(const actuators::PropulsorActuators& propulsors, const constants::Vector3_T<T>& center_of_gravity, const dynamics::Twist_T<T>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const actuators::PropulsorActuatorInputs_T<T>& u, const PropellerOmegaStateSet_T<T>& propeller_state);

    dynamics::Wrench_T<double> compute_propulsive_loads(const actuators::PropulsorActuators& propulsors, const constants::Vector3_T<double>& center_of_gravity, const dynamics::Twist_T<double>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const actuators::PropulsorActuatorInputs_T<double>& u, const PropellerOmegaStateSet_T<double>& propeller_state);

}

#include "simulation/propulsion/public/detail/loads.tpp"
