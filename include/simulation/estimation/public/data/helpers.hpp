#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace estimation
{

	/**
	 * @brief Builds a rigid-body estimate from a measured state and reduced Kalman state.
	 *
	 * @param[in] Yt Measured rigid-body state supplying position and yaw.
	 * @param[in] zt Estimated reduced state vector.
	 * @return Rigid-body state containing the combined measured and estimated quantities.
	 */
	dynamics::RigidBodyState make_kalman_state_estimate(const dynamics::RigidBodyState& Yt,
		const dynamics::StateVector_T<double>& zt);

} // namespace estimation
