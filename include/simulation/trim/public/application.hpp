#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim
{

	/**
	 * @brief Applies a trimmed reduced state to a rigid-body state.
	 *
	 * @param[in] Xt Existing rigid-body state.
	 * @param[in] trim_state Trimmed velocity [m/s], angular velocity [rad/s], roll [rad], and pitch [rad].
	 * @return Updated rigid-body state.
	 */
	dynamics::RigidBodyState update_state_from_trim(const dynamics::RigidBodyState& Xt,
		const dynamics::State_T<double>& trim_state);

	/**
	 * @brief Initializes actuator lag states from a trim solution.
	 *
	 * @param[in,out] surface_actuators Surface actuators whose lag states are updated.
	 * @param[in,out] propulsor_actuators Propulsor actuators whose lag states are updated.
	 * @param[in] trim_sol Trim solution containing the actuator commands.
	 */
	void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators,
		actuators::PropulsorActuators& propulsor_actuators,
		const TrimSolution& trim_sol);

} // namespace trim
