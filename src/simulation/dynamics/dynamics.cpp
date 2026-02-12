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

    namespace common {
        const double dt = 0.01;

        Eigen::Vector3d f_cv(Eigen::Vector3d xt, Eigen::Vector3d xt_dot){
            return Eigen::Vector3d(0, 0, 0);
        }
    }


    std::array<Eigen::Vector3d, 2> fwd_euler(Eigen::Vector3d xt, Eigen::Vector3d xt_dot, DynamicsFunction f, double tf){
        int steps = static_cast<int>(std::floor(tf / common::dt));
        Eigen::Vector3d xt_ddot;

        for (int i = 0; i < steps; ++i){
            xt_ddot = f(xt, xt_dot);
            xt += xt_dot * common::dt;
            xt_dot += xt_ddot * common::dt;
        }
        return std::array<Eigen::Vector3d, 2>{xt, xt_dot};
    }

    
    EulerAngleRates wB_BI_2_eul_dot(const EulerAngles& eul, const BodyAxisRates& wB_BI) {
        Eigen::Matrix3d R;
        R <<    1,   std::sin(eul.phi()) * std::tan(eul.theta()),    std::cos(eul.phi()) * std::tan(eul.theta()),
                0,                           std::cos(eul.phi()),                           -std::sin(eul.phi()),
                0,  std::sin(eul.phi()) * global::sec(eul.theta()),   std::cos(eul.phi()) * global::sec(eul.theta());

        const Eigen::Vector3d eul_dot = R * wB_BI.data;
        return EulerAngleRates { eul_dot };
    }



    // TODO make sure every _kin function returns both velocity and position
    Position trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot){
        const Eigen::Vector3d xt1 = xt.data + xt_dot.data * common::dt + 0.5 * xt_ddot.data * (common::dt * common::dt);

        return Position { xt1 };
    }

    EulerAngles eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t){
        const Eigen::Vector3d eul_t1 = eul_t.data + eul_dot_t.data * common::dt;

        return EulerAngles { eul_t1 };
    }

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const BodyAxisRates& wB_BI){
        const Eigen::Matrix3d CIB_dot = -global::hat(wB_BI.data) * CIB.data;

        return OrientationMatrixRate { CIB_dot };
    }


    OrientationMatrix strap_down_kin(const OrientationMatrix& CIB_t, const BodyAxisRates& wB_BI_t){
        const double Omega = wB_BI_t.data.norm();
        Eigen::Matrix3d exp_term;

        if (Omega < global::eps) { 
            exp_term = global::I3; 
        } else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            Eigen::Matrix3d w_hat_skew = global::hat(w_hat);
            exp_term = global::I3 - std::sin(Omega * common::dt) * w_hat_skew + (1 - std::cos(Omega * common::dt)) * w_hat_skew * w_hat_skew;
        }

        const Eigen::Matrix3d CIB_t1 = exp_term * CIB_t.data;
        return OrientationMatrix { CIB_t1 };
    }

    OrientationQuaternionRate quat_kin(const OrientationQuaternion& qIB_t, const BodyAxisRates& wB_BI_t){
        const double Omega = wB_BI_t.data.norm();
        Eigen::Quaterniond dq;

        if (Omega < global::eps) {
            dq = Eigen::Quaterniond::Identity();
        } else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            dq.w() = std::cos(Omega * common::dt / 2);
            dq.vec() = -w_hat * std::sin(Omega * common::dt / 2);
        }

        const Eigen::Quaterniond qIB_t1 = dq * qIB_t.data;
        return OrientationQuaternionRate { transforms::normalize_and_canonicalize(qIB_t1) };

    }



















    OrientationMatrix HomogenousFrameTransformationMatrix::C() const { return OrientationMatrix { transforms::RfromH(data) }; }
    Position HomogenousFrameTransformationMatrix::p() const { return Position { transforms::dfromH(data) }; }
    void HomogenousFrameTransformationMatrix::set(const Eigen::Matrix3d& C, const Eigen::Vector3d& p){ data = transforms::makeHC(C, p, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const Eigen::Matrix3d& C){ data = transforms::makeHC(C, p().data, "translate"); }
    void HomogenousFrameTransformationMatrix::set(const Eigen::Vector3d& p){ data = transforms::makeHC(C().data, p, "translate"); }

    double EulerAngles::phi() const   { return data[0]; }
    double EulerAngles::theta() const { return data[1]; }
    double EulerAngles::psi() const   { return data[2]; }

    double BodyAxisRates::p() const { return data[0]; }
    double BodyAxisRates::q() const { return data[1]; }
    double BodyAxisRates::r() const { return data[2]; }

    double EulerAngleRates::phi_dot() const   { return data[0]; }
    double EulerAngleRates::theta_dot() const { return data[1]; }
    double EulerAngleRates::psi_dot() const   { return data[2]; }

    void OrientationQuaternion::set(const Eigen::Matrix3d& C) { data = transforms::C }
    void OrientationQuaternion::set(double a, double b, double c, const std::string& order, const std::string& type) { transforms::eul2quatC(a, b, c, order, type) }


    BodyAxisRates OrientationQuaternionRate::wB_BI() const { return BodyAxisRates { data.vec() }; }






}