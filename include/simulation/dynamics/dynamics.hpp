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
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/transforms/transforms.hpp"

namespace frames { struct Frame; } // forward declare

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
    struct Wrench;
    struct Twist;

    struct Position {
        Eigen::Vector3d data; // e.g. pI_BI
    };

    struct OrientationMatrix {
        Eigen::Matrix3d data; // e.g. CIB
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul);
    };

    struct HomogenousFrameTransformationMatrix {
        Eigen::Matrix4d data; // H
        OrientationMatrix C() const;
        Position          p() const;
        void set(const OrientationMatrix& C, const Position& p);
        void set(const OrientationMatrix& C);
        void set(const Position& p);
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul);
    };

    struct OrientationQuaternion {
        Eigen::Quaterniond data; // e.g. qIB
        // void set(double a, double b, double c, const std::string& order, const std::string& type);
        void set(const OrientationMatrix& C);
        void set(const EulerAngles& eul);
    };

    struct EulerAngles {
        Eigen::Vector3d data;   // eul ("ZYX", intrinsic). Stored as [z, y, x]
        double psi() const;     // yaw
        double theta() const;   // pitch
        double phi() const;     // roll
        void set(const OrientationMatrix& C);
        void set(const OrientationQuaternion& q);

    };

    struct OrientationMatrixRate {
        Eigen::Matrix3d data; // e.g. CIB_dot
        void set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C);
        void set(const OrientationMatrix& C, const AngularVelocity& w);
    };

    struct OrientationQuaternionRate {
        Eigen::Quaterniond data; // e.g. qIB_dot
        void set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q);
        void set(const OrientationQuaternion& q, const AngularVelocity& w);
    };

   struct AngularVelocity {
        Eigen::Vector3d data; // e.g. wB_BI
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
        Eigen::Quaterniond data; // e.g. wq_BI = [ 0; wB_BI ]
        AngularVelocity w() const;
        // void set(const Eigen::Vector3d& wB_BI);
        void set(const AngularVelocity& w);

    };

    struct LinearVelocity {
        Eigen::Vector3d data; // e.g. vB_BI
    };

    struct LinearAcceleration {
        Eigen::Vector3d data; // e.g. vB_BI_dot
    };

    struct AngularAcceleration {
        Eigen::Vector3d data; // e.g. wB_BI_dot
    };

    struct VerticalSpeed {
        double data;   // alt_BE_dot [m/s]
    };

    struct Force {
        Eigen::Vector3d data; // F [N]
    };

    struct SpecificForce {
        Eigen::Vector3d data; // F/m [ms^-2]
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
        dynamics::Position p;               // e.g. pI_BI
        dynamics::LinearVelocity v;         // e.g. vB_BI
        dynamics::OrientationQuaternion q;  // e.g. qIB
        dynamics::AngularVelocity w;        // e.g. wB_BI
    };

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    RigidBodyState rigid_body_state(const frames::Frame& F);

    struct Wrench {
        dynamics::Force F;          
        dynamics::Moment M;         
    };

    struct Twist {
        dynamics::LinearVelocity v;         // e.g. vB_BI
        dynamics::AngularVelocity w;        // e.g. wB_BI
    };

    template <typename T>
    struct Twist_T {
        constants::Vector3_T<T> v = constants::Zero3_T<T>;
        constants::Vector3_T<T> w = constants::Zero3_T<T>;
    };

    template <typename T>
    constants::Matrix3_T<T> _eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Matrix3_T<T> _wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> _ddtB_vB_BI_T(const constants::Vector3_T<T>& vB, const constants::Vector3_T<T>& wB_BI, double mass, const constants::Vector3_T<T>& FB_net);

    template <typename T>
    constants::Vector3_T<T> _ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net);

    template <typename T>
    constants::Vector3_T<T> _ddtB_to_ddtI_T(const constants::Vector3_T<T>& ddtB_v, const constants::Vector3_T<T>& v, const constants::Vector3_T<T>& w);

    template <typename T>
    constants::Vector3_T<T> _eul_dot_to_wB_BI_T(const constants::Vector3_T<T>& eul_dot, const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> _wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi);



    /** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame
        In this codebase, 'inertial' is used as a strict synonym for the ECEF frame
        However, as implemented in this codebase, the NED frame is technically also an inertial frame, so the NED frame could technically also be used with these functions
        This is because, as implemented in this codebase, the NED frame remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial behavior)
        For simplicity, however, we strictly assume that 'I' and 'inertial' refer to the ECEF frame in the below dynamics functions
        This may change in the future, but, for now, always assume inertial <=> ECEF and NEVER pass arguments corresponding to other frames to the below dynamics functions
    */

    EulerAngleRates _wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);
    Position _trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    LinearVelocity _trans_kin_vel(const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    EulerAngles _eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t);

    OrientationMatrixRate _ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);
    OrientationMatrixRate _ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);
    OrientationMatrix _rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternion _quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternionRate _quat_kin_vel(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);

    /**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
    LinearAcceleration _ddtB_vB_BI(const LinearVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);
    
    /**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
    Eigen::Vector3d _ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net);


    /**
    * @brief Converts a body derivative to an inertial derivative
    */
    Eigen::Vector3d _ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w);


    LinearVelocity _trans_dyn_vel(const LinearVelocity& vB_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t);
    AngularVelocity _rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t);
    RigidBodyState step_rigid_body(const RigidBodyState& xB_BI_t, const Mass& mass, const InertiaTensor& J, const Force& FB_net_t, const Moment& MB_net_t);

    AngularVelocity _CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);
    OrientationQuaternionRate _CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);
    AngularVelocity _qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);
    OrientationMatrixRate _qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);
    AngularVelocity _eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);
    EulerAngleRates _wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);

    Eigen::Matrix3d _eul_dot_to_wB_BI_mat(double theta, double phi);

    Eigen::Matrix3d _wB_BI_to_eul_dot_mat(double theta, double phi);
}

#include "simulation/dynamics/dynamics.tpp"
