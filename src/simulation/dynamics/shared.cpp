#include "simulation/dynamics/shared.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/transforms/transforms.hpp"

namespace dynamics {

    void OrientationMatrix::set(const OrientationQuaternion& q) { data = transforms::quat_to_rot(q.data); }
    void OrientationMatrix::set(const EulerAngles& eul) { data = transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"); }

    OrientationMatrix HomogenousFrameTransformationMatrix::C() const { return OrientationMatrix{ transforms::C_from_H(data) }; }
    Position HomogenousFrameTransformationMatrix::p() const { return Position{ transforms::p_from_H(data) }; }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { data = transforms::make_HC(C.data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C) { data = transforms::make_HC(C.data, p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const Position& p) { data = transforms::make_HC(C().data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationQuaternion& q) { data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const EulerAngles& eul) { data = transforms::make_HC(transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); }

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

}
