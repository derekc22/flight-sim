#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <vector>
#include <tuple>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace dynamics {

    /** @deprecated */
    // Eigen::Vector3d f_cv(Eigen::Vector3d xt, Eigen::Vector3d xt_dot){
    //     return Eigen::Vector3d(0, 0, 0);
    // }

    Eigen::Matrix3d _eul_dot_to_wB_BI_mat(double theta, double phi){
        return _eul_dot_to_wB_BI_mat_T(theta, phi);
    }

    Eigen::Matrix3d _wB_BI_to_eul_dot_mat(double theta, double phi) {
        return _wB_BI_to_eul_dot_mat_T(theta, phi);
    }

    /** @deprecated */
    // std::array<Eigen::Vector3d, 2> fwd_euler(Eigen::Vector3d xt, Eigen::Vector3d xt_dot, DynamicsFunction f, double tf){
    //     int steps = static_cast<int>(std::floor(tf / constants::dt));
    //     Eigen::Vector3d xt_ddot;

    //     for (int i = 0; i < steps; ++i){
    //         xt_ddot = f(xt, xt_dot);
    //         xt += xt_dot * constants::dt;
    //         xt_dot += xt_ddot * constants::dt;
    //     }
    //     return std::array<Eigen::Vector3d, 2>{xt, xt_dot};
    // }




    // TODO make sure every _kin function returns both velocity and position
    Position _trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot){
        const Eigen::Vector3d xt1 = xt.data + xt_dot.data * constants::dt + 0.5 * xt_ddot.data * (constants::dt * constants::dt);

        return { xt1 };
    }

    LinearVelocity _trans_kin_vel(const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot){
        const Eigen::Vector3d xt_dot1 = xt_dot.data + xt_ddot.data * constants::dt;

        return { xt_dot1 };
    }

    EulerAngles _eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t){
        const Eigen::Vector3d eul_t1 = eul_t.data + eul_dot_t.data * constants::dt;

        return { eul_t1 };
    }

    OrientationMatrixRate _ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI){
        const Eigen::Matrix3d CIB_dot = -util::hat(wB_BI.data) * CIB.data;

        return { CIB_dot };
    }

    OrientationMatrixRate _ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI){
        const Eigen::Matrix3d CBI_dot = CBI.data * util::hat(wB_BI.data);

        return { CBI_dot };
    }


    OrientationMatrix _rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t){
        // strap_down/poisson
        const double Omega = wB_BI_t.data.norm();
        Eigen::Matrix3d exp_term;

        if (Omega < constants::eps) exp_term = constants::I3; 
        else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            Eigen::Matrix3d w_hat_skew = util::hat(w_hat);
            exp_term = constants::I3 - std::sin(Omega * constants::dt) * w_hat_skew + (1 - std::cos(Omega * constants::dt)) * w_hat_skew * w_hat_skew;
        }

        const Eigen::Matrix3d CIB_t1 = exp_term * CIB_t.data;
        return { CIB_t1 };
    }

    OrientationQuaternion _quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t){
        const double Omega = wB_BI_t.data.norm();
        Eigen::Quaterniond dq;

        if (Omega < constants::eps) dq = Eigen::Quaterniond::Identity();
        else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            dq.w() = std::cos(Omega * constants::dt / 2);
            dq.vec() = -w_hat * std::sin(Omega * constants::dt / 2);  // minus for qIB convention
        }

        const Eigen::Quaterniond qIB_t1 = dq * qIB_t.data;
        return { transforms::normalize_and_canonicalize(qIB_t1) };
    }


    OrientationQuaternionRate _quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI) {
        Eigen::Quaterniond wq_BI;
        wq_BI.w() = 0.0;
        wq_BI.vec() = - wB_BI.data;                  // minus for qIB convention
        Eigen::Quaterniond qIB_dot = wq_BI * qIB.data;
        qIB_dot.coeffs() *= 0.5;
        return { qIB_dot };  // do NOT canonicalize qIB_dot
    }

    OrientationQuaternionRate _quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocityQuaternion& wq_BI){
        OrientationQuaternionRate qIB_dot = _quat_kin_vel(qIB, wq_BI.w());
        return qIB_dot;
    }


    // a) Translational Dynamics (non-rotating earth), velocity expressed in body coordinates
    //
    // vB_dot = (1/mass) * FB_net - wB_BI x vB
    //
    // Inputs at t0: FB_net(t0), wB_BI(t0), vB(t0)
    // Output: vB(t0 + dt)
    LinearAcceleration _ddtB_vB_BI(const LinearVelocity& vB_BI, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net){
        return { _ddtB_vB_BI_T<double>(vB_BI.data, wB_BI.data, mass.data, FB_net.data) };
    }

    LinearVelocity _trans_dyn_vel(const LinearVelocity& vB_BI_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t){
        const LinearAcceleration vB_BI_dot_t = _ddtB_vB_BI(vB_BI_t, wB_BI_t, mass, FB_net_t);

        const Eigen::Vector3d vB_BI_t1 = vB_BI_t.data + vB_BI_dot_t.data * constants::dt;
        return { vB_BI_t1 };
    }

    Eigen::Vector3d _ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w){
        return _ddtB_to_ddtI_T<double>(ddtB_v, v, w);
    }


    // b) Rotational Dynamics, body rates expressed in body coordinates
    //
    // J * w_dot + w x (J*w) = M
    // w_dot = J^{-1} * (M - w x (J*w))
    //
    // Inputs at t0: M(t0), J, w(t0)
    // Output: w(t0 + dt)
    Eigen::Vector3d _ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net){
        return _ddtB_wB_BI_T<double>(wB_BI.data, J.data, MB_net.data);
    }

    AngularVelocity _rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t){
        const Eigen::Vector3d wB_BI_dot_t = _ddtB_wB_BI(wB_BI_t, J, MB_net_t);

        const Eigen::Vector3d wB_BI_t1 = wB_BI_t.data + wB_BI_dot_t * constants::dt;
        return { wB_BI_t1 };
    }



    RigidBodyState step_rigid_body(const RigidBodyState& xB_BI_t, const Mass& mass, const InertiaTensor& J, const Force& FB_net_t, const Moment& MB_net_t){

        // Yes, this is structurally correct now. 
        // ddtB_vB_BI_t is the body derivative of velocity expressed in the body frame, 
        // ddtI_vB_BI_t is the inertial derivative of velocity expressed in the body frame, 
        // and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of velocity expressed in the inertial frame for your inertial-position kinematics update. 
        // This assumes CBI maps body components to inertial components 
        // and that _ddtB_vB_BI returns the body derivative of velocity expressed in the body frame

        const Eigen::Matrix3d CIB_t = transforms::quat_to_rot(xB_BI_t.q.data);
        const Eigen::Matrix3d CBI_t = CIB_t.transpose();

        // Translational dynamics in body coordinates
        const LinearVelocity vB_BI_t1 = _trans_dyn_vel(xB_BI_t.v, xB_BI_t.w, mass, FB_net_t); 
        const Eigen::Vector3d ddtB_vB_BI_t = _ddtB_vB_BI(xB_BI_t.v, xB_BI_t.w, mass, FB_net_t).data;           // produces a body derivative
        const Eigen::Vector3d ddtI_vB_BI_t = _ddtB_to_ddtI(ddtB_vB_BI_t, xB_BI_t.v.data, xB_BI_t.w.data);      // produces an inertial derivative
        const LinearAcceleration aB_BI_t{ ddtI_vB_BI_t }; // since pI_BI_t1 and vI_BI_t are inertial, aB_BI_t needs to be an inertial derivative

        // Rotational dynamics in body coordinates
        const AngularVelocity wB_BI_t1 = _rot_dyn(xB_BI_t.w, J, MB_net_t);

        // Quaternion rotational kinematics
        const OrientationQuaternion qIB_t1 = _quat_kin(xB_BI_t.q, xB_BI_t.w);

        // Convert body velocity/acceleleration to inertial for translational kinematics update on pI_BI
        const Eigen::Vector3d vI_BI_t = CBI_t * xB_BI_t.v.data;
        const Eigen::Vector3d aI_BI_t = CBI_t * aB_BI_t.data;

        // Translational kinematics in inertial coordinates
        const Position pI_BI_t1 = _trans_kin(xB_BI_t.p, LinearVelocity { vI_BI_t }, LinearAcceleration { aI_BI_t });

        return { pI_BI_t1, vB_BI_t1, qIB_t1, wB_BI_t1 };
    }


    AngularVelocity _CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB){
        Eigen::Matrix3d Omega_skew =  - CIB_dot.data * CIB.data.transpose();         // minus because CIB convention
        Omega_skew = 0.5 * (Omega_skew - Omega_skew.transpose());                    // enforce skew
        Eigen::Vector3d wB_BI = util::vee(Omega_skew);

        return { wB_BI };
    }

    OrientationQuaternionRate _CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB){
        AngularVelocity wB_BI = _CIB_dot_to_wB_BI(CIB_dot, CIB) ;
        OrientationQuaternionRate qIB_dot = _quat_kin_vel(qIB, wB_BI);

        return qIB_dot;
    }

    AngularVelocity _qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB){
        Eigen::Quaterniond qdot_qinv = qIB_dot.data * qIB.data.conjugate();
        qdot_qinv.coeffs() *= -2; // minus for qIB convention
        Eigen::Vector3d wB_BI = qdot_qinv.vec();

        return { wB_BI };
    }

    OrientationQuaternionRate _wB_BI_to_qIB_dot(const AngularVelocity& wB_BI, const OrientationQuaternion& qIB){
        Eigen::Quaterniond wq(0.0, wB_BI.data.x(), wB_BI.data.y(), wB_BI.data.z());
        Eigen::Quaterniond qdot = wq * qIB.data;
        qdot.coeffs() *= -0.5; // minus for qIB convention
        return { qdot };
    }

    OrientationMatrixRate _qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB){
        AngularVelocity wB_BI = _qIB_dot_to_wB_BI(qIB_dot, qIB);
        Eigen::Matrix3d CIB_dot = - util::hat(wB_BI.data) * CIB.data; // minus for qIB convention

        return { CIB_dot };
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









    void OrientationMatrix::set(const OrientationQuaternion& q){ data = transforms::quat_to_rot(q.data); }
    void OrientationMatrix::set(const EulerAngles& eul) { data = transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"); }

    OrientationMatrix HomogenousFrameTransformationMatrix::C() const { return OrientationMatrix { transforms::C_from_H(data) }; }
    Position HomogenousFrameTransformationMatrix::p() const { return Position { transforms::p_from_H(data) }; }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { data = transforms::make_HC(C.data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C) { data = transforms::make_HC(C.data, p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const Position& p) { data = transforms::make_HC(C().data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationQuaternion& q){ data = transforms::make_HC(transforms::quat_to_rot(q.data), p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const EulerAngles& eul) { data = transforms::make_HC(transforms::eul_to_C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); }

    // void OrientationQuaternion::set(double a, double b, double c, const std::string& order, const std::string& type) { data = transforms::eul_to_quatC(a, b, c, order, type); }
    void OrientationQuaternion::set(const OrientationMatrix& C) { data = transforms::normalize_and_canonicalize(transforms::rot_to_quat(C.data)); }
    void OrientationQuaternion::set(const EulerAngles& eul) { data = transforms::normalize_and_canonicalize(transforms::eul_to_quatC(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr")); }

    double EulerAngles::psi() const   { return data[0]; }
    double EulerAngles::theta() const { return data[1]; }
    double EulerAngles::phi() const   { return data[2]; }
    void EulerAngles::set(const OrientationMatrix& C) { data = transforms::C_to_eul(C.data, "ZYX", "intr"); }
    void EulerAngles::set(const OrientationQuaternion& q) { data = transforms::quatC_to_eul(q.data, "ZYX", "intr"); }

    void OrientationMatrixRate::set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C) { data = dynamics::_qIB_dot_to_CIB_dot(q_dot, q, C).data; }
    void OrientationMatrixRate::set(const OrientationMatrix& C, const AngularVelocity& w) { data = dynamics::_ddt_CIB(C, w).data; }

    void OrientationQuaternionRate::set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q) { data = dynamics::_CIB_dot_to_qIB_dot(C_dot, C, q).data; };
    void OrientationQuaternionRate::set(const OrientationQuaternion& q, const AngularVelocity& w) { data = dynamics::_quat_kin_vel(q, w).data; }

    
    double AngularVelocity::p() const { return data[0]; }
    double AngularVelocity::q() const { return data[1]; }
    double AngularVelocity::r() const { return data[2]; }

    double EulerAngleRates::phi_dot() const   { return data[0]; }
    double EulerAngleRates::theta_dot() const { return data[1]; }
    double EulerAngleRates::psi_dot() const   { return data[2]; }
    void EulerAngleRates::set(const AngularVelocity& w, const EulerAngles& eul){ data = dynamics::_wB_BI_to_eul_dot(w, eul).data; }

    AngularVelocity AngularVelocityQuaternion::w() const { return AngularVelocity { data.vec() }; }
    void AngularVelocityQuaternion::set(const AngularVelocity& w) { Eigen::Quaterniond q; q.w() = 0; q.vec() = w.data; data = q; }







}
