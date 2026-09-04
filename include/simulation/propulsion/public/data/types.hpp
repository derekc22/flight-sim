#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <optional>

namespace propulsion
{

	template <typename T> struct PropellerOmegaState_T {
		T omega = T(0.0);
		T omega_dot = T(0.0);
	};

	template <typename T> struct PropellerOmegaStateSet_T {
		PropellerOmegaState_T<T> front_propulsor{};
		PropellerOmegaState_T<T> left_propulsor{};
		PropellerOmegaState_T<T> right_propulsor{};
	};

	template <typename T> struct PropulsionState_T {
		std::optional<T> front_propulsor_omega;
		std::optional<T> left_propulsor_omega;
		std::optional<T> right_propulsor_omega;
	};

	using PropulsionState = PropulsionState_T<double>;

	template <typename T> struct PropulsionManagerInput_T {
		const actuators::PropulsorActuators& propulsor_actuators;
		const constants::Vector3_T<T>& pB_GB;
		const dynamics::Twist_T<T>& twist;
		const atmospheric::StaticAtmosphericState& atm;
		const actuators::PropulsorActuatorInputs_T<T>& u;
		T dt;
		bool steady_state;
	};

	using PropulsionManagerInput = PropulsionManagerInput_T<double>;

	template <typename T> struct PropulsionManagerOutput_T {
		dynamics::Wrench_T<T> WB_propulsive;
		PropulsionState_T<T> propulsion_state_t;
	};

	using PropulsionManagerOutput = PropulsionManagerOutput_T<double>;

} // namespace propulsion
