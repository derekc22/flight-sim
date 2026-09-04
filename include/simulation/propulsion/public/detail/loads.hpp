#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	template <typename T>
	T compute_propeller_torque_T(const actuators::PropulsorActuator& propulsor,
		const T& omega,
		const atmospheric::AirDensity& rho);

	template <typename T>
	dynamics::Wrench_T<T> compute_propulsor_loads_T(const actuators::PropulsorActuator& propulsor,
		const constants::Vector3_T<T>& pB_GB,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const T& thrust,
		const PropellerOmegaState_T<T>& propeller_omega_state);

	template <typename T>
	dynamics::Wrench_T<T> compute_propulsive_loads_T(const actuators::PropulsorActuators& propulsor_actuators,
		const constants::Vector3_T<T>& pB_GB,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::PropulsorActuatorInputs_T<T>& u,
		const PropellerOmegaStateSet_T<T>& propeller_omega_state_set);

	dynamics::Wrench_T<double> compute_propulsive_loads(const actuators::PropulsorActuators& propulsor_actuators,
		const constants::Vector3_T<double>& pB_GB,
		const dynamics::Twist_T<double>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::PropulsorActuatorInputs_T<double>& u,
		const PropellerOmegaStateSet_T<double>& propeller_omega_state_set);

} // namespace propulsion

#include "simulation/propulsion/public/detail/loads.tpp"
