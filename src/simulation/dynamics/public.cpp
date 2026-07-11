#include <Eigen/Dense>
#include <format>
#include <stdexcept>
#include "simulation/dynamics/private.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/integrators/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/util/public.hpp"

namespace dynamics {

    void OrientationMatrix::set(const OrientationQuaternion& q) { 
        data = transforms::quat_to_rot(q.data); 
    }
    void OrientationMatrix::set(const EulerAngles& eul) { 
        data = transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"); 
    }

    OrientationMatrix HomogeneousTransformationMatrix::C() const { 
        return OrientationMatrix{ transforms::C_from_H(data) }; 
    }
    Position HomogeneousTransformationMatrix::p() const { 
        return Position{ transforms::p_from_H(data) }; 
    }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { 
        data = transforms::make_HC(C.data, p.data, "translate"); 
    }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C) {
        data = transforms::make_HC(C.data, p().data, "translate"); 
    }
    void HomogeneousTransformationMatrix::set(const Position& p) {
        data = transforms::make_HC(C().data, p.data, "translate"); 
    }
    void HomogeneousTransformationMatrix::set(const OrientationQuaternion& q) {
        data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, "translate"); 
    }
    void HomogeneousTransformationMatrix::set(const EulerAngles& eul) {
        data = transforms::make_HC(
            transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); 
    }

    void OrientationQuaternion::set(const OrientationMatrix& C) {
        data = transforms::normalize_and_canonicalize(transforms::rot_to_quat(C.data)); 
    }
    void OrientationQuaternion::set(const EulerAngles& eul) {
        data = transforms::normalize_and_canonicalize(
            transforms::eul_to_quatC(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr")
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
        data = transforms::C_to_eul(C.data, "ZYX", "intr"); 
    }
    void EulerAngles::set(const OrientationQuaternion& q) { 
        data = transforms::quatC_to_eul(q.data, "ZYX", "intr"); 
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

    RigidBodyState compute_rigid_body_state(const frames::Frame& F) {
        if (F.parent != nullptr && F.parent->name != "NEDFrameECEF") {
            throw std::invalid_argument(
                std::format(
                    "rigid_body_state: Invalid frame input. "
                    "The parent of {} must be an inertial frame: ECEFFrame or NEDFrameECEF", F.name
                )
            );
        }
        const frames::FrameView fv = F.view();
        return {
            .p = fv.H->p(),
            .v = *fv.v,
            .q = *fv.q,
            .w = *fv.w
        };
    }

    AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { eul_dot_to_wB_BI_mat(theta, phi) * eul_dot.data };
    }

    State_T<double> pack_state(const StateVector& x) {
        return {
            .vx = x(0),
            .vy = x(1),
            .vz = x(2),
            .p = x(3),
            .q = x(4),
            .r = x(5),
            .phi = x(6),
            .theta = x(7)
        };
    }

    StateVector unpack_state(const State_T<double>& x) {
        StateVector out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta;
        return out;
    }

    State_T<double> pack_state(const RigidBodyState& Xt) {
        TranslationalVelocity vB_BI = Xt.v;
        AngularVelocity wB_BI = Xt.w;
        EulerAngles eulIB;
        eulIB.set(Xt.q);

        return {
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
    }

    StateVector unpack_state(const RigidBodyState& Xt) {
        return unpack_state(pack_state(Xt));
    }

}
