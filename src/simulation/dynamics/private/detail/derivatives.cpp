#include <Eigen/Dense>
#include "simulation/dynamics/private/detail/derivatives.hpp"
#include "simulation/util/linalg/public.hpp"
#include "simulation/util/trig/public.hpp"

namespace dynamics {

    Eigen::Matrix3d eul_dot_to_wB_BI_mat(double theta, double phi) {
        Eigen::Matrix3d T_mat;
        T_mat << 1.0, 0.0, -util::sin(theta),
                 0.0, util::cos(phi), util::sin(phi) * util::cos(theta),
                 0.0, -util::sin(phi), util::cos(phi) * util::cos(theta);
        return T_mat;
    }

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI) {
        const Eigen::Matrix3d CIB_dot = -util::hat(wB_BI.data) * CIB.data;  // minus for qIB convention

        return { CIB_dot };
    }

    OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI) {
        const Eigen::Matrix3d CBI_dot = CBI.data * util::hat(wB_BI.data);

        return { CBI_dot };
    }

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI) {
        Eigen::Quaterniond wq_BI;
        wq_BI.w() = 0.0;
        wq_BI.vec() = - wB_BI.data; // minus for qIB convention
        Eigen::Quaterniond qIB_dot = wq_BI * qIB.data;
        qIB_dot.coeffs() *= 0.5;
        return { qIB_dot };  // do NOT canonicalize qIB_dot
    }

    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocityQuaternion& wq_BI) {
        OrientationQuaternionRate qIB_dot = quat_kin_vel(qIB, wq_BI.w());
        return qIB_dot;
    }

    OrientationQuaternionRate CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB) {
        AngularVelocity wB_BI = CIB_dot_to_wB_BI(CIB_dot, CIB) ;
        OrientationQuaternionRate qIB_dot = quat_kin_vel(qIB, wB_BI);

        return qIB_dot;
    }

    OrientationQuaternionRate wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB) {
        Eigen::Quaterniond wq(0.0, wB_BI.data.x(), wB_BI.data.y(), wB_BI.data.z());
        Eigen::Quaterniond qdot = wq * qIB.data;
        qdot.coeffs() *= -0.5; // minus for qIB convention
        return { qdot };
    }

    OrientationMatrixRate qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB) {
        AngularVelocity wB_BI = qIB_dot_to_wB_BI(qIB_dot, qIB);
        Eigen::Matrix3d CIB_dot = - util::hat(wB_BI.data) * CIB.data; // minus for qIB convention

        return { CIB_dot };
    }

}
