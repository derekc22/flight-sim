#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace dynamics {

    /** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame (i.e. the ECEF frame or NED frame)
        Note that the NED frame is assumed inertial in this codebase as it remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial)
        In practice, most of the calls made in this codebase to the functions below are WRT the NED frame; however, as stated above, calls WRT the ECEF frame are also valid
    */

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);

    OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI);

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocityQuaternion& wq_BI);

    /**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
    TranslationalAcceleration ddtB_vB_BI(const TranslationalVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);
    
    /**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
    Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& JB, const Moment& MB_net);

    /**
    * @brief Converts a body derivative to an inertial derivative
    */
    Eigen::Vector3d ddtB_to_ddtI(const Eigen::Vector3d& ddtB_vB, const Eigen::Vector3d& vB, const Eigen::Vector3d& wB_BI);

    /**
    * @brief Converts an inertial derivative to a body derivative
    */
    Eigen::Vector3d ddtI_to_ddtB(const Eigen::Vector3d& ddtI_vI, const Eigen::Vector3d& vI, const Eigen::Vector3d& wI_BI);

    OrientationQuaternionRate CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);

    OrientationQuaternionRate wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB);

    OrientationMatrixRate qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);

    Eigen::Matrix3d eul_dot_to_wB_BI_mat(double theta, double phi);

}
