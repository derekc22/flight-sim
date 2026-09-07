#pragma once
#include "simulation/constants/public/scalars.hpp"
#include "simulation/propulsion/public/components/effector.hpp"
#include "simulation/util/public/math.hpp"

namespace propulsion
{

	template <typename T>
	PropulsorEffectorOutput_T<T> PropulsorEffector::step(
		const PropulsorEffectorInput_T<T>& input)
	{
		dynamics::Wrench_T<T> WB_propulsive;
		const constants::Vector3_T<T> n_prop_t = n_prop.cast<T>();
		const constants::Vector3_T<T> pG_propG = pB_propB.cast<T>() - input.pB_GB;

		WB_propulsive.F = n_prop_t * input.thrust;
		WB_propulsive.M = pG_propG.cross(WB_propulsive.F);

		if (!propellers.has_value()) {
			return {.WB_propulsive = WB_propulsive, .propeller_omega = {}};
		}

		const PropellerAssembly& propeller_assembly = propellers.value();
		const double omega_denominator = propeller_assembly.thrust_coeff * input.rho.data *
			propeller_assembly.diameter * propeller_assembly.diameter * propeller_assembly.diameter *
			propeller_assembly.diameter;
		const T propeller_omega =
			T(2.0 * constants::pi) * util::sqrt((input.thrust + T(constants::eps)) / T(omega_denominator));
		const T propeller_omega_dot = !input.steady_state && propeller_omega_t_1.has_value()
			? (propeller_omega - T(propeller_omega_t_1.value())) / input.dt
			: T(0.0);

		const T n = propeller_omega / T(2.0 * constants::pi);
		const T propeller_torque = T(propeller_assembly.torque_coeff * input.rho.data * propeller_assembly.diameter *
									   propeller_assembly.diameter * propeller_assembly.diameter *
									   propeller_assembly.diameter * propeller_assembly.diameter) *
			n * n;
		const constants::Vector3_T<T> propeller_angular_momentum =
			n_prop_t * (T(propeller_assembly.spin_inertia * propeller_assembly.spin_sign) * propeller_omega);

		WB_propulsive.M += -input.wB_BI.cross(propeller_angular_momentum);
		WB_propulsive.M +=
			-n_prop_t * (T(propeller_assembly.spin_inertia * propeller_assembly.spin_sign) * propeller_omega_dot);
		WB_propulsive.M += -n_prop_t * (T(propeller_assembly.spin_sign) * propeller_torque);

		return {.WB_propulsive = WB_propulsive, .propeller_omega = propeller_omega};
	}

} // namespace propulsion
