#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/integrators/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Computes concrete aerodynamic, propulsive, and net wrenches for an RK4 stage.
	 *
	 * Converts inertial wind and NED gravity to body axes before evaluating the templated wrench model.
	 *
	 * @param[in] Xt Rigid-body state at the stage.
	 * @param[in,out] model Structural state and aerodynamic and propulsion managers used during evaluation.
	 * @param[in] conditions Atmospheric, inertial-wind, and steady-state conditions.
	 * @param[in] u Actuator inputs for the stage.
	 * @param[in] dt Time step used by the propulsion model [s].
	 * @return Aerodynamic, propulsive, and net body-frame wrenches plus the evaluated propulsion state.
	 */
	WrenchEvaluation compute_net_wrench(const dynamics::RigidBodyState& Xt,
		RK4Model& model,
		const operating::OperatingConditions& conditions,
		const actuators::ActuatorInputs_T<double>& u,
		double dt);

} // namespace integrators
