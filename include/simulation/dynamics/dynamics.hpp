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

    struct Position;
    struct OrientationMatrix;
    struct HomogenousFrameTransformationMatrix;
    struct OrientationQuaternion;
    struct EulerAngles;
    struct OrientationMatrixRate;
    struct OrientationQuaternionRate;
    struct AngularVelocity;
    struct EulerAngleRates;
    struct AngularVelocityQuaternion;
    struct LinearVelocity;
    struct LinearAcceleration;
    struct Force;
    struct Gravity;
    struct InertiaTensor;
    struct Mass;
    struct CenterOfGravity;
    struct Moment;

    using DynamicsFunction =
    std::function<Eigen::Vector3d(
        const Eigen::Vector3d&,
        const Eigen::Vector3d&
    )>;

    struct Position {
        Eigen::Vector3d data; // pI_BI
    };

    struct OrientationMatrix {
        Eigen::Matrix3d data; // CIB
    };

    struct HomogenousFrameTransformationMatrix {
        Eigen::Matrix4d data; // H
        OrientationMatrix C() const;
        Position          p() const;
        void set(const OrientationMatrix& C, const Position& p);
        void set(const OrientationMatrix& C);
        void set(const Position& p);
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul, const std::string& order);
    };

    struct OrientationQuaternion {
        Eigen::Quaterniond data; // qIB
        // void set(double a, double b, double c, const std::string& order, const std::string& type);
        void set(const OrientationMatrix& C);
        void set(const EulerAngles& eul, const std::string& order);
    };

    struct EulerAngles {
        Eigen::Vector3d data; // eul ("ZYX", intrinsic)
        double phi() const;
        double theta() const;
        double psi() const;
        void set(const OrientationMatrix& C, const std::string& order);
        void set(const OrientationQuaternion& q, const std::string& order);

    };

    struct OrientationMatrixRate {
        Eigen::Matrix3d data; // CIB_dot
        void set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C);
        void set(const OrientationMatrix& C, const AngularVelocity& w);
    };

    struct OrientationQuaternionRate {
        Eigen::Quaterniond data; // qIB_dot
        // void set(const Eigen::Matrix3d& C_dot, const Eigen::Matrix3d& C, const Eigen::Quaterniond& q);
        // void set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q);
        void set(const OrientationQuaternion& q, const AngularVelocity& w);
    };

   struct AngularVelocity {
        Eigen::Vector3d data; // wB_BI
        double p() const;
        double q() const;
        double r() const;
    };

    struct EulerAngleRates {
        Eigen::Vector3d data; // eul_dot ("ZYX", intrinsic)
        double phi_dot() const;
        double theta_dot() const;
        double psi_dot() const;
        void set(const AngularVelocity& w, const EulerAngles& eul);

    };

    struct AngularVelocityQuaternion {
        Eigen::Quaterniond data; // wq_BI = [ 0; wB_BI ]
        AngularVelocity wB_BI() const;
        // void set(const Eigen::Vector3d& wB_BI);
        void set(const AngularVelocity& wB_BI);

    };

    struct LinearVelocity {
        Eigen::Vector3d data; // pB_BI_dot
    };

    struct LinearAcceleration {
        Eigen::Vector3d data; // pB_BI_ddot
    };

    struct Force {
        Eigen::Vector3d data; // F [N]
    };

    struct Gravity {
        Eigen::Vector3d data; // g [ms^-2]
    };

    struct InertiaTensor {
        Eigen::Matrix3d data; // J
    };

    struct Mass {
        double data; // m [kg]
    };

    struct Moment {
        Eigen::Vector3d data; // M [Nm]
    };

    struct CenterOfGravity {
        Eigen::Vector3d data; // CG [m]
    };

    struct RigidBodyState {
        // Must genuinely store values WRT the inertial frame (ECEF) ONLY
        Position pI_BI;
        LinearVelocity vB_BI;
        OrientationQuaternion qIB;
        AngularVelocity wB_BI;
    };





    
    std::array<Eigen::Vector3d, 2> fwd_euler(Eigen::Vector3d xt, Eigen::Vector3d xt_dot, DynamicsFunction f, double tf);
    EulerAngleRates wB_BI_2_eul_dot(const EulerAngles& eul, const AngularVelocity& wB_BI);
    Position trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    EulerAngles eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t);

    OrientationMatrixRate ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);
    OrientationMatrix rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternion quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternionRate quat_kin_vel(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);

    /**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
    LinearAcceleration ddtB_vB_BI(const LinearVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);
    
    /**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
    Eigen::Vector3d ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net);


    /**
    * @brief Converts a body derivative to an inertial derivative
    */
    Eigen::Vector3d ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w);


    LinearVelocity trans_dyn_vel(const LinearVelocity& vB_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t);
    AngularVelocity rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t);
    RigidBodyState step_rigid_body(const RigidBodyState& xt, const Mass& mass, const InertiaTensor& J, const Force& FB_net_t, const Moment& MB_net_t);

    AngularVelocity CIB_dot2wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);
    OrientationQuaternionRate CIB_dot2qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);
    AngularVelocity qIB_dot2wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);
    OrientationMatrixRate qIB_dot2CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);
    AngularVelocity eul_dot2wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);
    EulerAngleRates wB_BI2eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);


    namespace common {
        const extern double dt; // s

        Eigen::Vector3d f_cv(Eigen::Vector3d xt, Eigen::Vector3d xt_dot); // constant velocity

        Eigen::Matrix3d eul_dot2wB_BI_mat(double theta, double phi);

        Eigen::Matrix3d wB_BI2eul_dot_mat(double theta, double phi);


    }

}
