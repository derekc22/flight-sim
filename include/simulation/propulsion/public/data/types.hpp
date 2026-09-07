#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <optional>

namespace propulsion
{
	struct PropellerAssembly {
		double spin_sign;
		double thrust_coeff;
		double torque_coeff;
		double diameter;
		double spin_inertia;
	};

	template <typename T> struct PropulsorEffectorInput_T {
		const constants::Vector3_T<T>& pB_GB;
		const constants::Vector3_T<T>& wB_BI;
		const atmospheric::AirDensity& rho;
		const T& thrust;
		T dt;
		bool steady_state;
	};

	template <typename T> struct PropulsorEffectorOutput_T {
		dynamics::Wrench_T<T> WB_propulsive;
		std::optional<T> propeller_omega;
	};

	template <typename T> struct PropulsionState_T {
		std::optional<T> front_propulsor_omega;
		std::optional<T> left_propulsor_omega;
		std::optional<T> right_propulsor_omega;
	};

	using PropulsionState = PropulsionState_T<double>;

	template <typename T> struct PropulsionManagerInput_T {
		const constants::Vector3_T<T>& pB_GB;
		const dynamics::Twist_T<T>& twist;
		const atmospheric::StaticAtmosphericState& atm;
		const actuators::PropulsorActuatorInputs_T<T>& u;
		T dt;
		bool steady_state;
	};

	template <typename T> struct PropulsionManagerOutput_T {
		dynamics::Wrench_T<T> WB_propulsive;
		PropulsionState_T<T> propulsion_state_t;
	};

} // namespace propulsion
