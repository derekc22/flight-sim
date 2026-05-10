#include <cmath>
#include <format>
#include <stdexcept>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/util/util.hpp"

namespace dynamics {

    RigidBodyState compute_rigid_body_state(const frames::Frame& F) {
        if (F.parent != nullptr && F.parent->name != "NEDFrameECEF") {
            throw std::invalid_argument(std::format("dynamics::rigid_body_state: Invalid frame input, the parent of {} must be an inertial frame: ECEFFrame or NEDFrameECEF", F.name));
        }
        const frames::FrameView fv = F.view();
        return {
            .p = fv.H->p(),
            .v = *fv.v,
            .q = *fv.q,
            .w = *fv.w
        };
    }

    Position _trans_kin(const Position& xt, const TranslationalVelocity& xt_dot, const TranslationalAcceleration& xt_ddot){
        const Eigen::Vector3d xt1 = xt.data + xt_dot.data * constants::dt + 0.5 * xt_ddot.data * (constants::dt * constants::dt);

        return { xt1 };
    }

    TranslationalVelocity _trans_kin_vel(const TranslationalVelocity& xt_dot, const TranslationalAcceleration& xt_ddot){
        const Eigen::Vector3d xt_dot1 = xt_dot.data + xt_ddot.data * constants::dt;

        return { xt_dot1 };
    }

    OrientationQuaternion _quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t){
        const double Omega = wB_BI_t.data.norm();
        Eigen::Quaterniond dq;

        if (Omega < constants::eps) dq = constants::qI;
        else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            dq.w() = util::cos(Omega * constants::dt / 2);
            dq.vec() = -w_hat * util::sin(Omega * constants::dt / 2);
        }

        const Eigen::Quaterniond qIB_t1 = dq * qIB_t.data;
        return { transforms::normalize_and_canonicalize(qIB_t1) };
    }

    void OrientationMatrix::set(const OrientationQuaternion& q) { data = transforms::quat_to_rot(q.data); }
    void OrientationMatrix::set(const EulerAngles& eul) { data = transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"); }

    OrientationMatrix HomogeneousFrameTransformationMatrix::C() const { return OrientationMatrix{ transforms::C_from_H(data) }; }
    Position HomogeneousFrameTransformationMatrix::p() const { return Position{ transforms::p_from_H(data) }; }
    void HomogeneousFrameTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { data = transforms::make_HC(C.data, p.data, "translate"); }
    void HomogeneousFrameTransformationMatrix::set(const OrientationMatrix& C) { data = transforms::make_HC(C.data, p().data, "translate"); }
    void HomogeneousFrameTransformationMatrix::set(const Position& p) { data = transforms::make_HC(C().data, p.data, "translate"); }
    void HomogeneousFrameTransformationMatrix::set(const OrientationQuaternion& q) { data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, "translate"); }
    void HomogeneousFrameTransformationMatrix::set(const EulerAngles& eul) { data = transforms::make_HC(transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); }

    void OrientationQuaternion::set(const OrientationMatrix& C) { data = transforms::normalize_and_canonicalize(transforms::rot_to_quat(C.data)); }
    void OrientationQuaternion::set(const EulerAngles& eul) { data = transforms::normalize_and_canonicalize(transforms::eul_to_quatC(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr")); }

    double EulerAngles::psi() const { return data[0]; }
    double EulerAngles::theta() const { return data[1]; }
    double EulerAngles::phi() const { return data[2]; }
    void EulerAngles::set(const OrientationMatrix& C) { data = transforms::C_to_eul(C.data, "ZYX", "intr"); }
    void EulerAngles::set(const OrientationQuaternion& q) { data = transforms::quatC_to_eul(q.data, "ZYX", "intr"); }

    void OrientationMatrixRate::set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C) { data = dynamics::_qIB_dot_to_CIB_dot(q_dot, q, C).data; }
    void OrientationMatrixRate::set(const OrientationMatrix& C, const AngularVelocity& w) { data = dynamics::_ddt_CIB(C, w).data; }

    void OrientationQuaternionRate::set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q) { data = dynamics::_CIB_dot_to_qIB_dot(C_dot, C, q).data; }
    void OrientationQuaternionRate::set(const OrientationQuaternion& q, const AngularVelocity& w) { data = dynamics::_quat_kin_vel(q, w).data; }

    double AngularVelocity::p() const { return data[0]; }
    double AngularVelocity::q() const { return data[1]; }
    double AngularVelocity::r() const { return data[2]; }

    double EulerAngleRates::phi_dot() const { return data[0]; }
    double EulerAngleRates::theta_dot() const { return data[1]; }
    double EulerAngleRates::psi_dot() const { return data[2]; }
    void EulerAngleRates::set(const AngularVelocity& w, const EulerAngles& eul) { data = dynamics::_wB_BI_to_eul_dot(w, eul).data; }

    AngularVelocity AngularVelocityQuaternion::w() const { return AngularVelocity{ data.vec() }; }
    void AngularVelocityQuaternion::set(const AngularVelocity& w) {
        Eigen::Quaterniond q;
        q.w() = 0;
        q.vec() = w.data;
        data = q;
    }

    AngularVelocity _CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB){
        Eigen::Matrix3d Omega_skew =  - CIB_dot.data * CIB.data.transpose();
        Omega_skew = 0.5 * (Omega_skew - Omega_skew.transpose());
        Eigen::Vector3d wB_BI = util::vee(Omega_skew);

        return { wB_BI };
    }

    AngularVelocity _qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB){
        Eigen::Quaterniond qdot_qinv = qIB_dot.data * qIB.data.conjugate();
        qdot_qinv.coeffs() *= -2;
        Eigen::Vector3d wB_BI = qdot_qinv.vec();

        return { wB_BI };
    }

    AngularVelocity _eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { _eul_dot_to_wB_BI_T<double>(eul_dot.data, theta, phi) };
    }

    EulerAngleRates _wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { _wB_BI_to_eul_dot_T<double>(wB_BI.data, theta, phi) };
    }

}
