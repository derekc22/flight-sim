#include <Eigen/Dense>
#include <format>
#include <stdexcept>
#include "simulation/dynamics/private.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/util/public.hpp"

namespace dynamics {

    Position trans_kin(const Position& pt, const TranslationalVelocity& vt, const TranslationalAcceleration& at) {
        const Eigen::Vector3d pt1 = pt.data + vt.data * constants::dt + 0.5 * at.data * (constants::dt * constants::dt);

        return { pt1 };
    }

    OrientationQuaternion quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t){
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

    OrientationMatrix HomogeneousTransformationMatrix::C() const { return OrientationMatrix{ transforms::C_from_H(data) }; }
    Position HomogeneousTransformationMatrix::p() const { return Position{ transforms::p_from_H(data) }; }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { data = transforms::make_HC(C.data, p.data, "translate"); }
    void HomogeneousTransformationMatrix::set(const OrientationMatrix& C) { data = transforms::make_HC(C.data, p().data, "translate"); }
    void HomogeneousTransformationMatrix::set(const Position& p) { data = transforms::make_HC(C().data, p.data, "translate"); }
    void HomogeneousTransformationMatrix::set(const OrientationQuaternion& q) { data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, "translate"); }
    void HomogeneousTransformationMatrix::set(const EulerAngles& eul) { data = transforms::make_HC(transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); }

    void OrientationQuaternion::set(const OrientationMatrix& C) { data = transforms::normalize_and_canonicalize(transforms::rot_to_quat(C.data)); }
    void OrientationQuaternion::set(const EulerAngles& eul) { data = transforms::normalize_and_canonicalize(transforms::eul_to_quatC(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr")); }

    double EulerAngles::psi() const { return data[0]; }
    double EulerAngles::theta() const { return data[1]; }
    double EulerAngles::phi() const { return data[2]; }
    void EulerAngles::set(const OrientationMatrix& C) { data = transforms::C_to_eul(C.data, "ZYX", "intr"); }
    void EulerAngles::set(const OrientationQuaternion& q) { data = transforms::quatC_to_eul(q.data, "ZYX", "intr"); }

    void OrientationMatrixRate::set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C) { data = dynamics::qIB_dot_to_CIB_dot(q_dot, q, C).data; }
    void OrientationMatrixRate::set(const OrientationMatrix& C, const AngularVelocity& w) { data = dynamics::ddt_CIB(C, w).data; }

    void OrientationQuaternionRate::set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q) { data = dynamics::CIB_dot_to_qIB_dot(C_dot, C, q).data; }
    void OrientationQuaternionRate::set(const OrientationQuaternion& q, const AngularVelocity& w) { data = dynamics::quat_kin_vel(q, w).data; }

    double AngularVelocity::p() const { return data[0]; }
    double AngularVelocity::q() const { return data[1]; }
    double AngularVelocity::r() const { return data[2]; }

    double EulerAngleRates::phi_dot() const { return data[0]; }
    double EulerAngleRates::theta_dot() const { return data[1]; }
    double EulerAngleRates::psi_dot() const { return data[2]; }
    void EulerAngleRates::set(const AngularVelocity& w, const EulerAngles& eul) { data = dynamics::wB_BI_to_eul_dot(w, eul).data; }

    AngularVelocity AngularVelocityQuaternion::w() const { return AngularVelocity{ data.vec() }; }
    void AngularVelocityQuaternion::set(const AngularVelocity& w) {
        Eigen::Quaterniond q;
        q.w() = 0;
        q.vec() = w.data;
        data = q;
    }

    AngularVelocity CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB){
        Eigen::Matrix3d Omega_skew =  - CIB_dot.data * CIB.data.transpose();
        Omega_skew = 0.5 * (Omega_skew - Omega_skew.transpose());
        Eigen::Vector3d wB_BI = util::vee(Omega_skew);

        return { wB_BI };
    }

    AngularVelocity qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB){
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

    TranslationalVelocity trans_kin_vel(const TranslationalVelocity& vt, const TranslationalAcceleration& at) {
        const Eigen::Vector3d vt1 = vt.data + at.data * constants::dt;

        return { vt1 };
    }

    AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        return { eul_dot_to_wB_BI_mat(theta, phi) * eul_dot.data };
    }

    State_T<double> pack_state(const RigidBodyState& Xt){
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

    StateVector_T<double> unpack_state(const RigidBodyState& Xt){
        return unpack_state_T(pack_state(Xt));
    }

    RigidBodyState step_rigid_body(const RigidBodyState& XB_BI_t, const Mass& mass, const InertiaTensor& J, const Wrench& WB_net_t){

        // ddtB_vB_BI_t is the body derivative of body-expressed velocity, 
        // ddtI_vB_BI_t is the inertial derivative of body-expressed velocity, 
        // and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of inertial-expressed velocity for the inertial-expressed position update
        // Where CBI maps body components to inertial components 
        // and ddtB_vB_BI returns the body derivative of body-expressed velocity

        const dynamics::Force FB_net_t = WB_net_t.F;
        const dynamics::Moment MB_net_t = WB_net_t.M;

        const Eigen::Matrix3d CIB_t = transforms::quat_to_rot(XB_BI_t.q.data);
        const Eigen::Matrix3d CBI_t = CIB_t.transpose();

        // Translational dynamics in body coordinates
        const TranslationalVelocity vB_BI_t1 = trans_dyn_vel(XB_BI_t.v, XB_BI_t.w, mass, FB_net_t);
        const Eigen::Vector3d ddtB_vB_BI_t = ddtB_vB_BI(XB_BI_t.v, XB_BI_t.w, mass, FB_net_t).data;           // produces a body derivative
        const Eigen::Vector3d ddtI_vB_BI_t = ddtB_to_ddtI(ddtB_vB_BI_t, XB_BI_t.v.data, XB_BI_t.w.data);      // produces an inertial derivative
        const TranslationalAcceleration aB_BI_t{ ddtI_vB_BI_t }; // since pI_BI_t1 and vI_BI_t are inertial, aB_BI_t needs to be an inertial derivative

        // Rotational dynamics in body coordinates
        const AngularVelocity wB_BI_t1 = rot_dyn(XB_BI_t.w, J, MB_net_t);

        // Quaternion rotational kinematics
        const OrientationQuaternion qIB_t1 = quat_kin(XB_BI_t.q, XB_BI_t.w);

        // Convert body velocity/acceleleration to inertial for translational kinematics update on pI_BI
        const Eigen::Vector3d vI_BI_t = CBI_t * XB_BI_t.v.data;
        const Eigen::Vector3d aI_BI_t = CBI_t * aB_BI_t.data;

        // Translational kinematics in inertial coordinates
        const Position pI_BI_t1 = trans_kin(XB_BI_t.p, TranslationalVelocity { vI_BI_t }, TranslationalAcceleration { aI_BI_t });

        return { pI_BI_t1, vB_BI_t1, qIB_t1, wB_BI_t1 };
    }

}
