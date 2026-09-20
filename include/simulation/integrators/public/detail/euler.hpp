#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Advances position using constant-acceleration translational kinematics.
	 *
	 * @param[in] pt Position at the current time [m].
	 * @param[in] vt Translational velocity at the current time [m/s].
	 * @param[in] at Translational acceleration at the current time [m/s^2].
	 * @param[in] dt Time step [s].
	 * @return Position after @p dt [m].
	 */
	dynamics::Position trans_kin(const dynamics::Position& pt,
		const dynamics::TranslationalVelocity& vt,
		const dynamics::TranslationalAcceleration& at,
		const double dt);

	/**
	 * @brief Advances an inertial-to-body quaternion using constant body angular velocity.
	 *
	 * @param[in] qIB_t Inertial-to-body orientation quaternion at the current time [-].
	 * @param[in] wB_BI_t Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] dt Time step [s].
	 * @return Inertial-to-body orientation quaternion after @p dt [-].
	 */
	dynamics::OrientationQuaternion
	quat_kin(const dynamics::OrientationQuaternion& qIB_t, const dynamics::AngularVelocity& wB_BI_t, const double dt);

	/**
	 * @brief Advances translational velocity using constant acceleration.
	 *
	 * @param[in] xt_dot Translational velocity at the current time [m/s].
	 * @param[in] xt_ddot Translational acceleration at the current time [m/s^2].
	 * @param[in] dt Time step [s].
	 * @return Translational velocity after @p dt [m/s].
	 */
	dynamics::TranslationalVelocity trans_kin_vel(const dynamics::TranslationalVelocity& xt_dot,
		const dynamics::TranslationalAcceleration& xt_ddot,
		const double dt);

	/**
	 * @brief Advances a rigid-body state by one integration step.
	 *
	 * @param[in] Xt Rigid-body state at the current time.
	 * @param[in] mass Vehicle mass [kg].
	 * @param[in] JB_G Body inertia tensor about the center of gravity [kg m^2].
	 * @param[in] WB_net_t Net body-expressed force [N] and moment [N m] at the current time.
	 * @param[in] dt Time step [s].
	 * @return Rigid-body state after @p dt.
	 */
	dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& Xt,
		const dynamics::Mass& mass,
		const dynamics::InertiaTensor& JB_G,
		const dynamics::Wrench& WB_net_t,
		const double dt);

} // namespace integrators
