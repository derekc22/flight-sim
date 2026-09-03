#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace dynamics {

	/** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame (i.e. the ECEF frame or NED frame)
        Note that the NED frame is assumed inertial in this codebase as it remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial)
        In practice, most of the calls made in this codebase to the functions below are WRT the NED frame; however, as stated above, calls WRT the ECEF frame are also valid
    */

	/**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
	TranslationalAcceleration
	ddtB_vB_BI(const TranslationalVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);

	/**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
	Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& JB, const Moment& MB_net);

	/**
    * @brief Converts a body derivative to an inertial derivative
    */
	Eigen::Vector3d
	ddtB_to_ddtI(const Eigen::Vector3d& ddtB_vB, const Eigen::Vector3d& vB, const Eigen::Vector3d& wB_BI);

	/**
    * @brief Converts an inertial derivative to a body derivative
    */
	Eigen::Vector3d
	ddtI_to_ddtB(const Eigen::Vector3d& ddtI_vI, const Eigen::Vector3d& vI, const Eigen::Vector3d& wI_BI);

	template <typename T>
	constants::Vector3_T<T> ddtB_vB_BI_T(const constants::Vector3_T<T>& vB,
	    const constants::Vector3_T<T>& wB_BI,
	    double mass,
	    const constants::Vector3_T<T>& FB_net);

	template <typename T>
	constants::Vector3_T<T>
	ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net);

	template <typename T>
	constants::Vector3_T<T> wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi);

	template <typename T> constants::Matrix3_T<T> wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi);

	AngularVelocity CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);

	AngularVelocity qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);

	EulerAngleRates wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);

	AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);

} // namespace dynamics

#include "simulation/dynamics/public/detail/derivatives.tpp"
