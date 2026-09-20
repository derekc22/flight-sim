#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace dynamics
{

	/** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame (i.e. the ECEF frame or NED frame)
        Note that the NED frame is assumed inertial in this codebase as it remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial)
        In practice, most of the calls made in this codebase to the functions below are WRT the NED frame; however, as stated above, calls WRT the ECEF frame are also valid
    */

	/**
	 * @brief Computes the body derivative of body-expressed translational velocity.
	 *
	 * @param[in] vB Body-expressed translational velocity [m/s].
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame, expressed in body axes [rad/s].
	 * @param[in] mass Vehicle mass [kg].
	 * @param[in] FB_net Net force expressed in body axes [N].
	 * @return Body derivative of translational velocity expressed in body axes [m/s^2].
	 */
	TranslationalAcceleration
	ddtB_vB_BI(const TranslationalVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);

	/**
	 * @brief Computes the body derivative of body-expressed angular velocity.
	 *
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame, expressed in body axes [rad/s].
	 * @param[in] JB_G Inertia tensor about the center of gravity, expressed in body axes [kg m^2].
	 * @param[in] MB_net Net moment expressed in body axes [N m].
	 * @return Body angular acceleration expressed in body axes [rad/s^2].
	 */
	Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& JB_G, const Moment& MB_net);

	/**
	 * @brief Converts a derivative taken in body axes to an inertial derivative.
	 *
	 * @param[in] ddtB_vB Body derivative of a vector [m/s^2].
	 * @param[in] vB Vector whose derivative is converted [m/s].
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame [rad/s].
	 * @return Inertial derivative expressed in the same coordinate frame as the inputs [m/s^2].
	 */
	Eigen::Vector3d
	ddtB_to_ddtI(const Eigen::Vector3d& ddtB_vB, const Eigen::Vector3d& vB, const Eigen::Vector3d& wB_BI);

	/**
	 * @brief Converts an inertial derivative to a derivative taken in body axes.
	 *
	 * @param[in] ddtI_vI Inertial derivative of a vector [m/s^2].
	 * @param[in] vI Vector whose derivative is converted [m/s].
	 * @param[in] wI_BI Body angular velocity relative to an inertial frame [rad/s].
	 * @return Body derivative expressed in the same coordinate frame as the inputs [m/s^2].
	 */
	Eigen::Vector3d
	ddtI_to_ddtB(const Eigen::Vector3d& ddtI_vI, const Eigen::Vector3d& vI, const Eigen::Vector3d& wI_BI);

	/**
	 * @brief Computes the body derivative of body-expressed translational velocity.
	 *
	 * @tparam T Scalar type used for the calculation.
	 * @param[in] vB Body-expressed translational velocity [m/s].
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame, expressed in body axes [rad/s].
	 * @param[in] mass Vehicle mass [kg].
	 * @param[in] FB_net Net force expressed in body axes [N].
	 * @return Body derivative of translational velocity expressed in body axes [m/s^2].
	 */
	template <typename T>
	constants::Vector3_T<T> ddtB_vB_BI_T(const constants::Vector3_T<T>& vB,
		const constants::Vector3_T<T>& wB_BI,
		double mass,
		const constants::Vector3_T<T>& FB_net);

	/**
	 * @brief Computes the body derivative of body-expressed angular velocity.
	 *
	 * @tparam T Scalar type used for the calculation.
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame, expressed in body axes [rad/s].
	 * @param[in] JB_G Inertia tensor about the center of gravity, expressed in body axes [kg m^2].
	 * @param[in] MB_net Net moment expressed in body axes [N m].
	 * @return Body angular acceleration expressed in body axes [rad/s^2].
	 */
	template <typename T>
	constants::Vector3_T<T> ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI,
		const Eigen::Matrix3d& JB_G,
		const constants::Vector3_T<T>& MB_net);

	/**
	 * @brief Converts body angular velocity to intrinsic ZYX Euler-angle rates.
	 *
	 * @tparam T Scalar type used for the calculation.
	 * @param[in] wB_BI Body angular velocity relative to an inertial frame [rad/s].
	 * @param[in] theta Pitch angle [rad].
	 * @param[in] phi Roll angle [rad].
	 * @return Roll-, pitch-, and yaw-angle rates [rad/s].
	 */
	template <typename T>
	constants::Vector3_T<T> wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi);

	/**
	 * @brief Constructs the intrinsic ZYX mapping from body angular velocity to Euler-angle rates.
	 *
	 * @tparam T Scalar type used for the calculation.
	 * @param[in] theta Pitch angle [rad].
	 * @param[in] phi Roll angle [rad].
	 * @return Matrix mapping body angular velocity to roll-, pitch-, and yaw-angle rates [-].
	 */
	template <typename T> constants::Matrix3_T<T> wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi);

	/**
	 * @brief Computes body angular velocity from an orientation-matrix rate.
	 *
	 * @param[in] CIB_dot Orientation-matrix rate [1/s].
	 * @param[in] CIB Orientation matrix [-].
	 * @return Body angular velocity relative to the inertial frame [rad/s].
	 */
	AngularVelocity CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);

	/**
	 * @brief Computes body angular velocity from an orientation-quaternion rate.
	 *
	 * @param[in] qIB_dot Orientation-quaternion rate [1/s].
	 * @param[in] qIB Orientation quaternion [-].
	 * @return Body angular velocity relative to the inertial frame [rad/s].
	 */
	AngularVelocity qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);

	/**
	 * @brief Converts body angular velocity to intrinsic ZYX Euler-angle rates.
	 *
	 * @param[in] wB_BI Body angular velocity relative to the inertial frame [rad/s].
	 * @param[in] eul Intrinsic ZYX Euler angles [rad].
	 * @return Roll-, pitch-, and yaw-angle rates [rad/s].
	 */
	EulerAngleRates wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);

	/**
	 * @brief Converts intrinsic ZYX Euler-angle rates to body angular velocity.
	 *
	 * @param[in] eul_dot Roll-, pitch-, and yaw-angle rates [rad/s].
	 * @param[in] eul Intrinsic ZYX Euler angles [rad].
	 * @return Body angular velocity relative to the inertial frame [rad/s].
	 */
	AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);

} // namespace dynamics

#include "simulation/dynamics/public/detail/derivatives.tpp"
