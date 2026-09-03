#include <Eigen/Dense>
#include "simulation/dynamics/private/detail/derivatives.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/util/public/linalg.hpp"

namespace dynamics {

    TranslationalAcceleration ddtB_vB_BI(const TranslationalVelocity& vB_BI, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net) {
        return { ddtB_vB_BI_T<double>(vB_BI.data, wB_BI.data, mass.data, FB_net.data) };
    }

    Eigen::Vector3d ddtB_to_ddtI(const Eigen::Vector3d& ddtB_vB, const Eigen::Vector3d& vB, const Eigen::Vector3d& wB_BI) {
        return ddtB_vB + wB_BI.cross(vB);
    }

    Eigen::Vector3d ddtI_to_ddtB(const Eigen::Vector3d& ddtI_vI, const Eigen::Vector3d& vI, const Eigen::Vector3d& wI_BI) {
        return ddtI_vI - wI_BI.cross(vI);
    }

    Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& JB, const Moment& MB_net) {
        return ddtB_wB_BI_T<double>(wB_BI.data, JB.data, MB_net.data);
    }

    AngularVelocity CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB) {
        Eigen::Matrix3d Omega_skew =  - CIB_dot.data * CIB.data.transpose();
        Omega_skew = 0.5 * (Omega_skew - Omega_skew.transpose());
        Eigen::Vector3d wB_BI = util::vee(Omega_skew);

        return { wB_BI };
    }

    AngularVelocity qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB) {
        Eigen::Quaterniond qdot_qinv = qIB_dot.data * qIB.data.conjugate();
        qdot_qinv.coeffs() *= -2;
        Eigen::Vector3d wB_BI = qdot_qinv.vec();

        return { wB_BI };
    }

    EulerAngleRates wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { wB_BI_to_eul_dot_T<double>(wB_BI.data, theta, phi) };
    }

    AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { eul_dot_to_wB_BI_mat(theta, phi) * eul_dot.data };
    }

}
