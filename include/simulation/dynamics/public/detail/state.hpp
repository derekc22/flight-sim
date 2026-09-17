#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace frames
{
	struct Frame;
} // namespace frames

namespace dynamics
{

	/**
	 * @brief Returns the rigid-body state stored by a frame.
	 *
	 * @param[in] F Frame whose stored state is returned.
	 * @return Stored position, translational velocity, orientation, and angular velocity.
	 */
	RigidBodyState get_rigid_body_state(const frames::Frame& F);

	/**
	 * @brief Computes the rigid-body state of a frame relative to a reference frame.
	 *
	 * The result contains the position and orientation from @p R to @p F and the translational and angular velocity
	 * of @p F relative to @p R expressed in @p F.
	 *
	 * @param[in] F Frame whose relative state is computed.
	 * @param[in] R Reference frame.
	 * @return Rigid-body state of @p F relative to @p R.
	 */
	RigidBodyState compute_rigid_body_state(const frames::Frame& F, const frames::Frame& R);

	/**
	 * @brief Inverts a rigid-body state.
	 *
	 * @param[in] X_BA State of frame B relative to frame A.
	 * @return State of frame A relative to frame B.
	 */
	RigidBodyState invert_rigid_body_state(const RigidBodyState& X_BA);

	/**
	 * @brief Composes two consecutive rigid-body states.
	 *
	 * @param[in] X_BA State of frame B relative to frame A.
	 * @param[in] X_AR State of frame A relative to frame R.
	 * @return State of frame B relative to frame R.
	 */
	RigidBodyState compose_rigid_body_state(const RigidBodyState& X_BA, const RigidBodyState& X_AR);

	/**
	 * @brief Rebases a center-of-gravity rigid-body state to the body-frame origin.
	 *
	 * @param[in] X_GN State of the center-of-gravity frame relative to the NED frame.
	 * @param[in] pB_GB Position of the center of gravity relative to the body origin, expressed in body axes [m].
	 * @param[in] CGFrame Center-of-gravity frame providing its orientation and velocity relative to the body frame.
	 * @return State of the body-frame origin relative to the NED frame.
	 */
	RigidBodyState rebase_cg_state(const RigidBodyState& X_GN, const Position& pB_GB, const frames::Frame& CGFrame);

} // namespace dynamics
