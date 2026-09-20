#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/integrators/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Advances a rigid-body state using a fourth-order Runge-Kutta step.
	 *
	 * @param[in] Xt Rigid-body state at the current time.
	 * @param[in,out] model Structural state and aerodynamic and propulsion managers used during evaluation.
	 * @param[in] conditions Atmospheric, wind, and steady-state conditions.
	 * @param[in] u Actuator inputs for the step.
	 * @param[in] dt Time step [s].
	 * @return Rigid-body state after @p dt and the first-stage aerodynamic, propulsive, and net wrenches.
	 */
	RK4Output step_rigid_body_rk4(const dynamics::RigidBodyState& Xt,
		RK4Model& model,
		const operating::OperatingConditions& conditions,
		const actuators::ActuatorInputs_T<double>& u,
		double dt);

} // namespace integrators
