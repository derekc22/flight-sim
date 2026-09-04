#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/propulsion/public/detail/loads.hpp"

namespace propulsion
{

	template <typename T>
	T compute_propeller_torque_T(
		const actuators::PropulsorActuator& propulsor,
		const T& omega,
		const atmospheric::AirDensity& rho)
	{
		if (!propulsor.propellers) {
			return T(0.0);
		}
		const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
		const T n = omega / T(2.0 * constants::pi);
		return T(propellers.torque_coeff * rho.data * propellers.diameter * propellers.diameter * propellers.diameter *
				   propellers.diameter * propellers.diameter) *
			n * n;
	}

	template <typename T>
	dynamics::Wrench_T<T> compute_propulsor_loads_T(
		const actuators::PropulsorActuator& propulsor,
		const constants::Vector3_T<T>& pB_GB,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const T& thrust,
		const PropellerOmegaState_T<T>& propeller_omega_state)
	{
		dynamics::Wrench_T<T> out;

		const constants::Vector3_T<T>& wB_BI = twist.w;

		const constants::Vector3_T<T> n_prop = propulsor.n_prop.cast<T>();
		const constants::Vector3_T<T> pG_propG = propulsor.pB_propB.cast<T>() - pB_GB;
		const atmospheric::AirDensity& rho = atm.rho;

		out.F = n_prop * thrust;
		out.M = pG_propG.cross(out.F);

		if (propulsor.propellers.has_value()) {
			const actuators::PropellerAssembly& propellers = propulsor.propellers.value();
			const T q_prop = compute_propeller_torque_T<T>(propulsor, propeller_omega_state.omega, rho);
			const constants::Vector3_T<T> H_prop =
				n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * propeller_omega_state.omega);

			out.M += -wB_BI.cross(H_prop);
			out.M += -n_prop * (T(propellers.spin_inertia * propellers.spin_sign) * propeller_omega_state.omega_dot);
			out.M += -n_prop * (T(propellers.spin_sign) * q_prop);
		}

		return out;
	}

	template <typename T>
	dynamics::Wrench_T<T> compute_propulsive_loads_T(
		const actuators::PropulsorActuators& propulsor_actuators,
		const constants::Vector3_T<T>& pB_GB,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::PropulsorActuatorInputs_T<T>& u,
		const PropellerOmegaStateSet_T<T>& propeller_omega_state_set)
	{
		dynamics::Wrench_T<T> total;

		const dynamics::Wrench_T<T> front = compute_propulsor_loads_T<T>(propulsor_actuators.front_propulsor,
			pB_GB,
			twist,
			atm,
			u.front_propulsor_cmd,
			propeller_omega_state_set.front_propulsor);
		const dynamics::Wrench_T<T> left = compute_propulsor_loads_T<T>(propulsor_actuators.left_propulsor,
			pB_GB,
			twist,
			atm,
			u.left_propulsor_cmd,
			propeller_omega_state_set.left_propulsor);
		const dynamics::Wrench_T<T> right = compute_propulsor_loads_T<T>(propulsor_actuators.right_propulsor,
			pB_GB,
			twist,
			atm,
			u.right_propulsor_cmd,
			propeller_omega_state_set.right_propulsor);

		total.F = front.F + left.F + right.F;
		total.M = front.M + left.M + right.M;

		return total;
	}

} // namespace propulsion
