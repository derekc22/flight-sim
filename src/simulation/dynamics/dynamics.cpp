#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <vector>
#include <tuple>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"

namespace dynamics {

    Eigen::Vector3d f_cv(Eigen::Vector3d xt, Eigen::Vector3d xt_dot){
        return Eigen::Vector3d(0, 0, 0);
    }

    Eigen::Matrix3d _eul_dot2wB_BI_mat(double theta, double phi){
        Eigen::Matrix3d T;
        T << 1, 0, -std::sin(theta),
            0, std::cos(phi),  std::sin(phi) * std::cos(theta),
            0, -std::sin(phi), std::cos(phi) * std::cos(theta);
        return T;
    }

    Eigen::Matrix3d _wB_BI2eul_dot_mat(double theta, double phi) {
        Eigen::Matrix3d T;
        T <<    1,   std::sin(phi) * std::tan(theta),    std::cos(phi) * std::tan(theta),
                0,                           std::cos(phi),                           -std::sin(phi),
                0,  std::sin(phi) * global::sec(theta),   std::cos(phi) * global::sec(theta);
        return T;
    }

    std::array<Eigen::Vector3d, 2> fwd_euler(Eigen::Vector3d xt, Eigen::Vector3d xt_dot, DynamicsFunction f, double tf){
        int steps = static_cast<int>(std::floor(tf / global::dt));
        Eigen::Vector3d xt_ddot;

        for (int i = 0; i < steps; ++i){
            xt_ddot = f(xt, xt_dot);
            xt += xt_dot * global::dt;
            xt_dot += xt_ddot * global::dt;
        }
        return std::array<Eigen::Vector3d, 2>{xt, xt_dot};
    }




    // TODO make sure every _kin function returns both velocity and position
    Position _trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot){
        const Eigen::Vector3d xt1 = xt.data + xt_dot.data * global::dt + 0.5 * xt_ddot.data * (global::dt * global::dt);

        return Position { xt1 };
    }

    LinearVelocity trans_kin_vel(const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot){
        const Eigen::Vector3d xt_dot1 = xt_dot.data + xt_ddot.data * global::dt;

        return LinearVelocity { xt_dot1 };
    }

    EulerAngles _eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t){
        const Eigen::Vector3d eul_t1 = eul_t.data + eul_dot_t.data * global::dt;

        return EulerAngles { eul_t1 };
    }

    OrientationMatrixRate _ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI){
        const Eigen::Matrix3d CIB_dot = -global::hat(wB_BI.data) * CIB.data;

        return OrientationMatrixRate { CIB_dot };
    }

    OrientationMatrixRate ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI){
        const Eigen::Matrix3d CBI_dot = CBI.data * global::hat(wB_BI.data);

        return OrientationMatrixRate { CBI_dot };
    }


    OrientationMatrix _rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t){
        // strap_down/poisson
        const double Omega = wB_BI_t.data.norm();
        Eigen::Matrix3d exp_term;

        if (Omega < global::eps) exp_term = global::I3; 
        else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            Eigen::Matrix3d w_hat_skew = global::hat(w_hat);
            exp_term = global::I3 - std::sin(Omega * global::dt) * w_hat_skew + (1 - std::cos(Omega * global::dt)) * w_hat_skew * w_hat_skew;
        }

        const Eigen::Matrix3d CIB_t1 = exp_term * CIB_t.data;
        return OrientationMatrix { CIB_t1 };
    }

    OrientationQuaternion _quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t){
        const double Omega = wB_BI_t.data.norm();
        Eigen::Quaterniond dq;

        if (Omega < global::eps) {
            dq = Eigen::Quaterniond::Identity();
        } else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            dq.w() = std::cos(Omega * global::dt / 2);
            dq.vec() = -w_hat * std::sin(Omega * global::dt / 2);  // minus for qIB convention
        }

        const Eigen::Quaterniond qIB_t1 = dq * qIB_t.data;
        return OrientationQuaternion { transforms::normalize_and_canonicalize(qIB_t1) };
    }


    OrientationQuaternionRate _quat_kin_vel(const OrientationQuaternion& qIB, const AngularVelocity& wB_BI) {
        Eigen::Quaterniond wq_BI;
        wq_BI.w() = 0.0;
        wq_BI.vec() = - wB_BI.data;                  // minus for qIB convention
        Eigen::Quaterniond qIB_dot = wq_BI * qIB.data;
        qIB_dot.coeffs() *= 0.5;
        return OrientationQuaternionRate{ qIB_dot };  // do NOT canonicalize qIB_dot
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

        const Eigen::Vector3d w_cross_v = global::hat(wB_BI.data) * vB_BI.data;   // w x v
        const Eigen::Vector3d vB_BI_dot = (1.0 / mass.data) * FB_net.data - w_cross_v;

        return LinearAcceleration { vB_BI_dot };
    }

    LinearVelocity _trans_dyn_vel(const LinearVelocity& vB_BI_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t){
        const LinearAcceleration vB_BI_dot_t = _ddtB_vB_BI(vB_BI_t, wB_BI_t, mass, FB_net_t);

        const Eigen::Vector3d vB_BI_t1 = vB_BI_t.data + vB_BI_dot_t.data * global::dt;
        return LinearVelocity { vB_BI_t1 };
    }

    Eigen::Vector3d _ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w){
        const Eigen::Vector3d w_cross_v = global::hat(w) * v;
        const Eigen::Vector3d ddtI_v = ddtB_v + w_cross_v;
        return ddtI_v;
    }


    // b) Rotational Dynamics, body rates expressed in body coordinates
    //
    // J * w_dot + w x (J*w) = M
    // w_dot = J^{-1} * (M - w x (J*w))
    //
    // Inputs at t0: M(t0), J, w(t0)
    // Output: w(t0 + dt)
    Eigen::Vector3d _ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net){

        const Eigen::Vector3d w_cross_Jw = global::hat(wB_BI.data) * (J.data * wB_BI.data);       // w x (J*w)
        const Eigen::Vector3d rhs = MB_net.data - w_cross_Jw;

        return J.data.ldlt().solve(rhs);
    }

    AngularVelocity _rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t){
        const Eigen::Vector3d wB_BI_dot_t = _ddtB_wB_BI(wB_BI_t, J, MB_net_t);

        const Eigen::Vector3d wB_BI_t1 = wB_BI_t.data + wB_BI_dot_t * global::dt;
        return AngularVelocity { wB_BI_t1 };
    }



    RigidBodyState step_rigid_body(const RigidBodyState& xB_BI_t, const Mass& mass, const InertiaTensor& J, const Force& FB_net_t, const Moment& MB_net_t){

        // Yes, this is structurally correct now. 
        // ddtB_vB_BI_t is the body derivative of velocity expressed in the body frame, 
        // ddtI_vB_BI_t is the inertial derivative of velocity expressed in the body frame, 
        // and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of velocity expressed in the inertial frame for your inertial-position kinematics update. 
        // This assumes CBI maps body components to inertial components 
        // and that _ddtB_vB_BI returns the body derivative of velocity expressed in the body frame

        const Eigen::Matrix3d CIB_t = transforms::quat2rot(xB_BI_t.q.data);
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

        // Convert body velocity/accel to inertial for translational kinematics update on pI_BI
        const Eigen::Vector3d vI_BI_t = CBI_t * xB_BI_t.v.data;
        const Eigen::Vector3d aI_BI_t = CBI_t * aB_BI_t.data;

        // Translational kinematics in inertial coordinates
        const Position pI_BI_t1 = _trans_kin(xB_BI_t.p, LinearVelocity { vI_BI_t }, LinearAcceleration { aI_BI_t });

        return RigidBodyState { pI_BI_t1, vB_BI_t1, qIB_t1, wB_BI_t1 };
    }


    AngularVelocity _CIB_dot2wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB){
        Eigen::Matrix3d Omega_skew =  - CIB_dot.data * CIB.data.transpose();         // minus because CIB convention
        Omega_skew = 0.5 * (Omega_skew - Omega_skew.transpose());                    // enforce skew
        Eigen::Vector3d wB_BI = global::vee(Omega_skew);

        return AngularVelocity{ wB_BI };
    }

    OrientationQuaternionRate _CIB_dot2qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB){
        AngularVelocity wB_BI = _CIB_dot2wB_BI(CIB_dot, CIB) ;
        OrientationQuaternionRate qIB_dot = _quat_kin_vel(qIB, wB_BI);

        return qIB_dot;
    }

    AngularVelocity _qIB_dot2wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB){
        Eigen::Quaterniond qdot_qinv = qIB_dot.data * qIB.data.conjugate();
        qdot_qinv.coeffs() *= -2; // minus for qIB convention
        Eigen::Vector3d wB_BI = qdot_qinv.vec();

        return AngularVelocity{ wB_BI };
    }

    OrientationMatrixRate _qIB_dot2CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB){
        AngularVelocity wB_BI = _qIB_dot2wB_BI(qIB_dot, qIB);
        Eigen::Matrix3d CIB_dot = - global::hat(wB_BI.data) * CIB.data; // minus for qIB convention

        return OrientationMatrixRate{ CIB_dot };
    }

    AngularVelocity _eul_dot2wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        Eigen::Matrix3d T = _eul_dot2wB_BI_mat(theta, phi);
        Eigen::Vector3d wB_BI = T * eul_dot.data;
        
        return AngularVelocity { wB_BI };
    }

    EulerAngleRates _wB_BI2eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul) {
        double theta = eul.theta();
        double phi = eul.phi();
        Eigen::Matrix3d T = _wB_BI2eul_dot_mat(theta, phi);
        Eigen::Vector3d eul_dot = T * wB_BI.data;

        return EulerAngleRates { eul_dot };
    }











    OrientationMatrix HomogenousFrameTransformationMatrix::C() const { return OrientationMatrix { transforms::CfromH(data) }; }
    Position HomogenousFrameTransformationMatrix::p() const { return Position { transforms::pfromH(data) }; }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C, const Position& p) { data = transforms::makeHC(C.data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationMatrix& C) { data = transforms::makeHC(C.data, p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const Position& p) { data = transforms::makeHC(C().data, p.data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const OrientationQuaternion& q){ data = transforms::makeHC(transforms::quat2rot(q.data), p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const EulerAngles& eul) { data = transforms::makeHC(transforms::eul2C(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr"), p().data, "translate"); }

    // void OrientationQuaternion::set(double a, double b, double c, const std::string& order, const std::string& type) { data = transforms::eul2quatC(a, b, c, order, type); }
    void OrientationQuaternion::set(const OrientationMatrix& C) { data = transforms::normalize_and_canonicalize(transforms::rot2quat(C.data)); }
    void OrientationQuaternion::set(const EulerAngles& eul) { data = transforms::normalize_and_canonicalize(transforms::eul2quatC(eul.psi(), eul.theta(), eul.phi(), "ZYX", "intr")); }

    double EulerAngles::psi() const   { return data[0]; }
    double EulerAngles::theta() const { return data[1]; }
    double EulerAngles::phi() const   { return data[2]; }
    void EulerAngles::set(const OrientationMatrix& C) { data = transforms::C2eul(C.data, "ZYX", "intr"); }
    void EulerAngles::set(const OrientationQuaternion& q) { data = transforms::quatC2eul(q.data, "ZYX", "intr"); }

    void OrientationMatrixRate::set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C) { data = dynamics::_qIB_dot2CIB_dot(q_dot, q, C).data; }
    void OrientationMatrixRate::set(const OrientationMatrix& C, const AngularVelocity& w) { data = dynamics::_ddt_CIB(C, w).data; }

    void OrientationQuaternionRate::set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q) { data = dynamics::_CIB_dot2qIB_dot(C_dot, C, q).data; };
    void OrientationQuaternionRate::set(const OrientationQuaternion& q, const AngularVelocity& w) { data = dynamics::_quat_kin_vel(q, w).data; }

    
    double AngularVelocity::p() const { return data[0]; }
    double AngularVelocity::q() const { return data[1]; }
    double AngularVelocity::r() const { return data[2]; }

    double EulerAngleRates::phi_dot() const   { return data[0]; }
    double EulerAngleRates::theta_dot() const { return data[1]; }
    double EulerAngleRates::psi_dot() const   { return data[2]; }
    void EulerAngleRates::set(const AngularVelocity& w, const EulerAngles& eul){ data = dynamics::_wB_BI2eul_dot(w, eul).data; }

    AngularVelocity AngularVelocityQuaternion::w() const { return AngularVelocity { data.vec() }; }
    void AngularVelocityQuaternion::set(const AngularVelocity& w) { Eigen::Quaterniond q; q.w() = 0; q.vec() = w.data; data = q; }







}