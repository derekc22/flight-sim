#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <string>
#include <tuple> // For std::tuple
#include <array>
#include <functional>
#include "simulation/transforms/transforms.hpp"

namespace dynamics {

    using DynamicsFunction =
    std::function<Eigen::Vector3d(
        const Eigen::Vector3d&,
        const Eigen::Vector3d&
    )>;

    struct Position {
        Eigen::Vector3d data; // p
    };

    struct OrientationMatrix {
        Eigen::Matrix3d data; // CIB
    };

    struct HomogenousFrameTransformationMatrix {
        Eigen::Matrix4d data; // H
        OrientationMatrix C() const;
        Position          p() const;
        void set(const Eigen::Matrix3d& C, const Eigen::Vector3d& p);
        void set(const Eigen::Matrix3d& C)
        void set(const Eigen::Vector3d& p)
    };

    struct OrientationMatrixRate {
        Eigen::Matrix3d data; // CIB_dot
    };

    struct EulerAngles {
        Eigen::Vector3d data; // eul
        double phi() const;
        double theta() const;
        double psi() const;
    };

    struct BodyAxisRates {
        Eigen::Vector3d data; // wB_BI
        double p() const;
        double q() const;
        double r() const;
    };

    struct EulerAngleRates {
        Eigen::Vector3d data; // eul_dot
        double phi_dot() const;
        double theta_dot() const;
        double psi_dot() const;
    };

    struct OrientationQuaternion {
        Eigen::Quaterniond data; // qIB
        void set(const Eigen::Matrix3d& C);
        void set(double a, double b, double c, const std::string& order);
    };

    struct OrientationQuaternionRate {
        Eigen::Quaterniond data; // qB_BI_dot = [ 0; wB_BI ]
        BodyAxisRates wB_BI() const;
    };

    struct LinearVelocity {
        Eigen::Vector3d data; // pB_BI_dot
    };

    struct LinearAcceleration {
        Eigen::Vector3d data; // pB_BI_ddot
    };

    // struct AngularVelocity {
    //     Eigen::Vector3d _ang;
    // };

    // struct Attitude {
    //     EulerVector eulerVector;
    //     Quat quat
    // };

    // struct BodyRates {
    //     Eigen::Vector3d _ang;
    // };

    // struct Velocity {
    //     LinearVelocity;
    //     AngularVelocity;
    // };


    // struct Pose {
    //     Position pos;
    //     Attitude att;
    // }



    
    std::array<Eigen::Vector3d, 2> fwd_euler(Eigen::Vector3d xt, Eigen::Vector3d xt_dot, DynamicsFunction f, double tf);
    EulerAngleRates wB_BI_2_eul_dot(const EulerAngles& eul, const BodyAxisRates& wB_BI);
    Position trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    EulerAngles eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t);
    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const BodyAxisRates& wB_BI);
    OrientationMatrix strap_down_kin(const OrientationMatrix& CIB_t, const BodyAxisRates& wB_BI_t);
    OrientationQuaternionRate quat_kin(const OrientationQuaternion& qIB_t, const BodyAxisRates& wB_BI_t);



    namespace common {
        const extern double dt; // s

        Eigen::Vector3d f_cv(Eigen::Vector3d xt, Eigen::Vector3d xt_dot); // constant velocity

    }

}
