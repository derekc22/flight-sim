#include <Eigen/Dense>
#include "simulation/dynamics/private/detail/derivatives.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/transforms/public/detail/se3.hpp"
#include "simulation/transforms/public/detail/so3.hpp"

namespace dynamics {

    void OrientationMatrix::set(const OrientationQuaternion& q) { 
        data = transforms::quat_to_rot(q.data); 
    }
    void OrientationMatrix::set(const EulerAngles& eul) { 
        data = transforms::eul_to_C(
            eul.psi(), eul.theta(), eul.phi(), 
            transforms::EulerOrder::ZYX,
            transforms::RotationType::Intrinsic
        ); 
    }

    OrientationMatrix HomogeneousTransformationMatrix::C() const { 
        return OrientationMatrix{ transforms::C_from_H(data) }; 
    }
    Position HomogeneousTransformationMatrix::p() const { 
        return Position{ transforms::p_from_H(data) }; 
    }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { 
        data = transforms::make_HC(C.data, p.data, transforms::TransformationOrder::TranslateFirst); 
    }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C) {
        data = transforms::make_HC(C.data, p().data, transforms::TransformationOrder::TranslateFirst); 
    }
    void HomogeneousTransformationMatrix::set(const Position& p) {
        data = transforms::make_HC(C().data, p.data, transforms::TransformationOrder::TranslateFirst); 
    }
    void HomogeneousTransformationMatrix::set(const OrientationQuaternion& q) {
        data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, transforms::TransformationOrder::TranslateFirst); 
    }
    void HomogeneousTransformationMatrix::set(const EulerAngles& eul) {
        data = transforms::make_HC(
            transforms::eul_to_C(
                eul.psi(), eul.theta(), eul.phi(), 
                transforms::EulerOrder::ZYX, 
                transforms::RotationType::Intrinsic
            ), 
            p().data, 
            transforms::TransformationOrder::TranslateFirst
        ); 
    }

    void OrientationQuaternion::set(const OrientationMatrix& C) {
        data = transforms::rot_to_quat(C.data); 
    }
    void OrientationQuaternion::set(const EulerAngles& eul) {
        data = transforms::eul_to_quatC(
            eul.psi(), eul.theta(), eul.phi(), 
            transforms::EulerOrder::ZYX, 
            transforms::RotationType::Intrinsic
        );
    }

    double EulerAngles::psi() const { 
        return data[0]; 
    }
    double EulerAngles::theta() const { 
        return data[1]; 
    }
    double EulerAngles::phi() const { 
        return data[2]; 
    }
    void EulerAngles::set(const OrientationMatrix& C) {
        data = transforms::C_to_eul(C.data, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic); 
    }
    void EulerAngles::set(const OrientationQuaternion& q) { 
        data = transforms::quatC_to_eul(q.data, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic); 
    }

    void OrientationMatrixRate::set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C) {
        data = qIB_dot_to_CIB_dot(q_dot, q, C).data; 
    }
    void OrientationMatrixRate::set(const OrientationMatrix& C, const AngularVelocity& w) {
        data = ddt_CIB(C, w).data; 
    }

    void OrientationQuaternionRate::set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q) {
        data = CIB_dot_to_qIB_dot(C_dot, C, q).data; 
    }
    void OrientationQuaternionRate::set(const OrientationQuaternion& q, const AngularVelocity& w) {
        data = quat_kin_vel(q, w).data; 
    }

    double AngularVelocity::p() const {
        return data[0]; 
    }
    double AngularVelocity::q() const {
        return data[1]; 
    }
    double AngularVelocity::r() const {
        return data[2]; 
    }

    double EulerAngleRates::phi_dot() const {
        return data[0]; 
    }
    double EulerAngleRates::theta_dot() const {
        return data[1]; 
    }
    double EulerAngleRates::psi_dot() const {
        return data[2]; 
    }
    void EulerAngleRates::set(const AngularVelocity& w, const EulerAngles& eul) {
        data = wB_BI_to_eul_dot(w, eul).data; 
    }

    AngularVelocity AngularVelocityQuaternion::w() const { 
        return AngularVelocity{ data.vec() }; 
    }
    void AngularVelocityQuaternion::set(const AngularVelocity& w) {
        Eigen::Quaterniond q;
        q.w() = 0;
        q.vec() = w.data;
        data = q;
    }

}
