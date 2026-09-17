#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators
{

	/**
	 * @brief Advances intrinsic ZYX Euler angles using their current rates.
	 *
	 * @param[in] eul_t Euler angles at the current time [rad].
	 * @param[in] eul_dot_t Euler-angle rates at the current time [rad/s].
	 * @param[in] dt Time step [s].
	 * @return Euler angles after @p dt [rad].
	 */
	dynamics::EulerAngles
	eul_kin(const dynamics::EulerAngles& eul_t, const dynamics::EulerAngleRates& eul_dot_t, double dt);

	/**
	 * @brief Advances an inertial-to-body orientation matrix using constant angular velocity.
	 *
	 * Uses the strapdown rotation exponential with the sign required by the CIB convention.
	 *
	 * @param[in] CIB_t Inertial-to-body orientation matrix at the current time [-].
	 * @param[in] wB_BI_t Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] dt Time step [s].
	 * @return Inertial-to-body orientation matrix after @p dt [-].
	 */
	dynamics::OrientationMatrix
	rot_kin(const dynamics::OrientationMatrix& CIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

	/**
	 * @brief Advances body-expressed translational velocity using rigid-body dynamics.
	 *
	 * @param[in] vB_BI_t Body-expressed translational velocity at the current time [m/s].
	 * @param[in] wB_BI_t Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] mass Vehicle mass [kg].
	 * @param[in] FB_net_t Net force expressed in body axes [N].
	 * @param[in] dt Time step [s].
	 * @return Body-expressed translational velocity after @p dt [m/s].
	 */
	dynamics::TranslationalVelocity trans_dyn_vel(const dynamics::TranslationalVelocity& vB_BI_t,
		const dynamics::AngularVelocity& wB_BI_t,
		const dynamics::Mass& mass,
		const dynamics::Force& FB_net_t,
		double dt);

	/**
	 * @brief Advances body angular velocity using rigid-body rotational dynamics.
	 *
	 * @param[in] wB_BI_t Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] JB_G Body inertia tensor about the center of gravity [kg m^2].
	 * @param[in] MB_net_t Net moment expressed in body axes [N m].
	 * @param[in] dt Time step [s].
	 * @return Body angular velocity after @p dt [rad/s].
	 */
	dynamics::AngularVelocity rot_dyn(const dynamics::AngularVelocity& wB_BI_t,
		const dynamics::InertiaTensor& JB_G,
		const dynamics::Moment& MB_net_t,
		double dt);

} // namespace integrators
