#pragma once
#include "simulation/dynamics/public/detail/derivatives.hpp"

namespace dynamics
{

	/**
	 * @brief Computes the rate of an inertial-to-body orientation matrix.
	 *
	 * @param[in] CIB Inertial-to-body orientation matrix [-].
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @return Inertial-to-body orientation-matrix rate [1/s].
	 */
	OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);

	/**
	 * @brief Computes the rate of a body-to-inertial orientation matrix.
	 *
	 * @param[in] CBI Body-to-inertial orientation matrix [-].
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @return Body-to-inertial orientation-matrix rate [1/s].
	 */
	OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);

	/**
	 * @brief Computes orientation-quaternion rate using quaternion multiplication.
	 *
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI);

	/**
	 * @brief Computes orientation-quaternion rate using the Q(qIB) angular-velocity mapping.
	 *
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate quat_kin_vel_Q(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI);

	/**
	 * @brief Computes orientation-quaternion rate using the W(wB_BI) quaternion mapping.
	 *
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate quat_kin_vel_W(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI);

	/**
	 * @brief Computes orientation-quaternion rate from an angular-velocity quaternion.
	 *
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @param[in] wq_BI Pure quaternion storing `[0; wB_BI]` [rad/s].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocityQuaternion& wq_BI);

	/**
	 * @brief Converts an orientation-matrix rate to an orientation-quaternion rate.
	 *
	 * @param[in] CIB_dot Inertial-to-body orientation-matrix rate [1/s].
	 * @param[in] CIB Inertial-to-body orientation matrix [-].
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot,
		const OrientationMatrix& CIB,
		const OrientationQuaternion& qIB);

	/**
	 * @brief Converts body angular velocity to an orientation-quaternion rate.
	 *
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @return Inertial-to-body orientation-quaternion rate [1/s].
	 */
	OrientationQuaternionRate wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB);

	/**
	 * @brief Converts an orientation-quaternion rate to an orientation-matrix rate.
	 *
	 * @param[in] qIB_dot Inertial-to-body orientation-quaternion rate [1/s].
	 * @param[in] qIB Inertial-to-body orientation quaternion [-].
	 * @param[in] CIB Inertial-to-body orientation matrix [-].
	 * @return Inertial-to-body orientation-matrix rate [1/s].
	 */
	OrientationMatrixRate qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot,
		const OrientationQuaternion& qIB,
		const OrientationMatrix& CIB);

	/**
	 * @brief Constructs the intrinsic ZYX mapping from Euler-angle rates to body angular velocity.
	 *
	 * @param[in] theta Pitch angle [rad].
	 * @param[in] phi Roll angle [rad].
	 * @return Matrix mapping roll-, pitch-, and yaw-angle rates to body angular velocity [-].
	 */
	Eigen::Matrix3d eul_dot_to_wB_BI_mat(double theta, double phi);

} // namespace dynamics
