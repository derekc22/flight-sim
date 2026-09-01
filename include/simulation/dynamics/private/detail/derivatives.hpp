#pragma once
#include "simulation/dynamics/public/detail/derivatives.hpp"

namespace dynamics {

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);

    OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI);

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocityQuaternion& wq_BI);

    OrientationQuaternionRate CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);

    OrientationQuaternionRate wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB);

    OrientationMatrixRate qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);

    Eigen::Matrix3d eul_dot_to_wB_BI_mat(double theta, double phi);

}
