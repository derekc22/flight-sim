#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/integrators/public/detail/wrench.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace integrators
{

	template <typename T, typename Model>
	WrenchEvaluation_T<T> compute_wrench_set_T(
		const Model& model,
		const dynamics::Twist_T<T>& twist,
		const atmospheric::StaticAtmosphericState& atm,
		const actuators::ActuatorInputs_T<T>& u,
		const atmospheric::Wind& windB,
		const constants::Vector3_T<T>& gB,
		T dt,
		bool steady_state)
	{
		const constants::Vector3_T<T> pB_GB = model.struc_t.pB_GB.data.template cast<T>();
		const aerodynamics::AerodynamicsManagerInput_T<T> aerodynamics_input{
			.pB_GB = pB_GB, .twist = twist, .atm = atm, .u = u.surface_inputs, .windB = windB
		};

		const aerodynamics::AerodynamicsManagerOutput_T<T> aerodynamics_output =
			model.aerodynamics_manager.step(aerodynamics_input);

		const dynamics::Wrench_T<T>& WB_aerodynamic = aerodynamics_output.WB_aerodynamic;

		const propulsion::PropulsionManagerInput_T<T> propulsion_input{
			.propulsor_actuators = model.propulsor_actuators,
			.pB_GB = pB_GB,
			.twist = twist,
			.atm = atm,
			.u = u.propulsor_inputs,
			.dt = dt,
			.steady_state = steady_state
		};

		const propulsion::PropulsionManagerOutput_T<T> propulsion_output =
			model.propulsion_manager.step(propulsion_input);

		const dynamics::Wrench_T<T>& WB_propulsive = propulsion_output.WB_propulsive;

		return {
			.WB_set =
				{.aerodynamic = WB_aerodynamic,
					.propulsive = WB_propulsive,
					.net =
						{.F = WB_aerodynamic.F + WB_propulsive.F + T(model.struc_t.mass.data) * gB,
							.M = WB_aerodynamic.M + WB_propulsive.M}},
			.propulsion_state_t = propulsion_output.propulsion_state_t
		};
	}

} // namespace integrators
