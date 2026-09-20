#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Computes the rigid-body state derivative for an RK4 stage.
	 *
	 * @param[in] Xt Rigid-body state at the stage.
	 * @param[in] mass Vehicle mass [kg].
	 * @param[in] JB_G Body inertia tensor about the center of gravity [kg m^2].
	 * @param[in] WB_net_t Net body-expressed force [N] and moment [N m].
	 * @return Inertial-expressed position rate [m/s], body velocity derivative [m/s^2], and body angular acceleration
	 * [rad/s^2].
	 */
	dynamics::RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& Xt,
		const dynamics::Mass& mass,
		const dynamics::InertiaTensor& JB_G,
		const dynamics::Wrench& WB_net_t);

	/**
	 * @brief Adds a scaled rigid-body state derivative to a state.
	 *
	 * @param[in] X Base rigid-body state.
	 * @param[in] X_dot Rigid-body state derivative.
	 * @param[in] scale Derivative scale [s].
	 * @return Scaled intermediate rigid-body state.
	 */
	dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X,
		const dynamics::RigidBodyStateDot& X_dot,
		const double scale);

	/**
	 * @brief Applies the RK4 weighted state derivative to a rigid-body state.
	 *
	 * @param[in] X Rigid-body state at the beginning of the step.
	 * @param[in] k1 First RK4 state derivative.
	 * @param[in] k2 Second RK4 state derivative.
	 * @param[in] k3 Third RK4 state derivative.
	 * @param[in] k4 Fourth RK4 state derivative.
	 * @param[in] dt Time step [s].
	 * @return Rigid-body state after the weighted RK4 update.
	 */
	dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X,
		const dynamics::RigidBodyStateDot& k1,
		const dynamics::RigidBodyStateDot& k2,
		const dynamics::RigidBodyStateDot& k3,
		const dynamics::RigidBodyStateDot& k4,
		const double dt);

} // namespace integrators
