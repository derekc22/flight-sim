#include "simulation/propulsion/public/detail/loads.hpp"

#include "simulation/constants/public/linalg.hpp"

namespace propulsion
{

	dynamics::Wrench_T<double> compute_propulsive_loads(
		const actuators::PropulsorActuators& propulsor_actuators,
		const constants::Vector3_T<double>& pB_GB,
		const dynamics::Twist_T<double>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::PropulsorActuatorInputs_T<double>& u,
		const PropellerOmegaStateSet_T<double>& propeller_omega_state_set)
	{
		return compute_propulsive_loads_T<double>(propulsor_actuators, pB_GB, twist, atm, u, propeller_omega_state_set);
	}

} // namespace propulsion
