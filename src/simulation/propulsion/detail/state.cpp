#include "simulation/propulsion/public/detail/state.hpp"

namespace propulsion
{

	PropellerOmegaStateSet_T<double> compute_propeller_omega_state_set(
		const actuators::PropulsorActuators& propulsor_actuators,
		const PropulsionState& propulsion_state_t_1,
		const actuators::PropulsorActuatorInputs_T<double>& u,
		const atmospheric::AirDensity& rho,
		double dt,
		bool steady_state)
	{
		return compute_propeller_omega_state_set_T<double>(
			propulsor_actuators, propulsion_state_t_1, u, rho, dt, steady_state);
	}

} // namespace propulsion
