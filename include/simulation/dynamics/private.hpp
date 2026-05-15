#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace dynamics {

    /** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame
        In this codebase, 'inertial' is used as a strict synonym for the ECEF frame
        However, as implemented in this codebase, the NED frame is technically also an inertial frame, so the NED frame could technically also be used with these functions
        This is because, as implemented in this codebase, the NED frame remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial behavior)
        For simplicity, however, we strictly assume that 'I' and 'inertial' refer to the ECEF frame in the below dynamics functions
        This may change in the future, but, for now, always assume inertial <=> ECEF and NEVER pass arguments corresponding to other frames to the below dynamics functions
    */

    EulerAngles eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t);

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);
    OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);
    OrientationMatrix rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);

    /**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
    TranslationalAcceleration ddtB_vB_BI(const TranslationalVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);
    
    /**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
    Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net);


    /**
    * @brief Converts a body derivative to an inertial derivative
    */
    Eigen::Vector3d ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w);


    TranslationalVelocity trans_dyn_vel(const TranslationalVelocity& vB_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t);
    AngularVelocity rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t);

    OrientationQuaternionRate CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);
    OrientationQuaternionRate wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB);
    OrientationMatrixRate qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);

    Eigen::Matrix3d eul_dot_to_wB_BI_mat(double theta, double phi);

    Eigen::Matrix3d wB_BI_to_eul_dot_mat(double theta, double phi);
}
