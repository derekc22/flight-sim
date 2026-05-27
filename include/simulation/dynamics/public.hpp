#pragma once
#include <Eigen/Dense>
#include "simulation/constants/public.hpp"

namespace frames { struct Frame; } // forward declare
namespace dynamics { struct OrientationQuaternion; struct EulerAngles; struct OrientationQuaternionRate; } // forward declare

namespace dynamics {

    struct Position {
        Eigen::Vector3d data;
    };

    struct OrientationMatrix {
        Eigen::Matrix3d data;
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul);
    };

    struct HomogeneousFrameTransformationMatrix {
        Eigen::Matrix4d data;
        OrientationMatrix C() const;
        Position p() const;
        void set(const OrientationMatrix& C, const Position& p);
        void set(const OrientationMatrix& C);
        void set(const Position& p);
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul);
    };

    struct OrientationQuaternion {
        Eigen::Quaterniond data;
        void set(const OrientationMatrix& C);
        void set(const EulerAngles& eul);
    };

    struct EulerAngles {
        Eigen::Vector3d data;
        double psi() const;
        double theta() const;
        double phi() const;
        void set(const OrientationMatrix& C);
        void set(const OrientationQuaternion& q);
    };

    struct AngularVelocity {
        Eigen::Vector3d data;
        double p() const;
        double q() const;
        double r() const;
    };

    struct OrientationMatrixRate {
        Eigen::Matrix3d data;
        void set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C);
        void set(const OrientationMatrix& C, const AngularVelocity& w);
    };

    struct OrientationQuaternionRate {
        Eigen::Quaterniond data;
        void set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q);
        void set(const OrientationQuaternion& q, const AngularVelocity& w);
    };

    struct EulerAngleRates {
        Eigen::Vector3d data;
        double phi_dot() const;
        double theta_dot() const;
        double psi_dot() const;
        void set(const AngularVelocity& w, const EulerAngles& eul);
    };

    struct AngularVelocityQuaternion {
        Eigen::Quaterniond data;
        AngularVelocity w() const;
        void set(const AngularVelocity& w);
    };

    struct TranslationalVelocity {
        Eigen::Vector3d data;
    };

    struct TranslationalAcceleration {
        Eigen::Vector3d data;
    };

    struct AngularAcceleration {
        Eigen::Vector3d data;
    };

    struct Force {
        Eigen::Vector3d data;
    };

    struct InertiaTensor {
        Eigen::Matrix3d data;
    };

    struct Mass {
        double data;
    };

    struct Moment {
        Eigen::Vector3d data;
    };

    struct RigidBodyState {
        Position p;
        TranslationalVelocity v;
        OrientationQuaternion q;
        AngularVelocity w;
    };

    struct Wrench {
        Force F;
        Moment M;
    };

    template <typename T>
    using StateVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    template <typename T>
    using StateDotVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    template <typename T>
    struct State_T {
        T vx = T(0);
        T vy = T(0);
        T vz = T(0);

        T p = T(0);
        T q = T(0);
        T r = T(0);

        T phi = T(0);
        T theta = T(0);
    };

    template <typename T>
    struct StateDot_T {
        T vx_dot = T(0);
        T vy_dot = T(0);
        T vz_dot = T(0);

        T p_dot = T(0);
        T q_dot = T(0);
        T r_dot = T(0);

        T phi_dot = T(0);
        T theta_dot = T(0);
    };

    template <typename T>
    StateVector_T<T> unpack_state_T(const State_T<T>& x);

    template <typename T>
    State_T<T> pack_state_vector(const StateVector_T<T>& z);

    template <typename T>
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot);

    template <typename T>
    constants::Vector3_T<T> ddtB_vB_BI_T(const constants::Vector3_T<T>& vB, const constants::Vector3_T<T>& wB_BI, double mass, const constants::Vector3_T<T>& FB_net);

    template <typename T>
    constants::Vector3_T<T> ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net);

    template <typename T>
    constants::Vector3_T<T> wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> eul_dot_to_wB_BI_T(const constants::Vector3_T<T>& eul_dot, const T& theta, const T& phi);

    template <typename T>
    constants::Matrix3_T<T> eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Matrix3_T<T> wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> ddtB_to_ddtI_T(const constants::Vector3_T<T>& ddtB_v, const constants::Vector3_T<T>& v, const constants::Vector3_T<T>& w);

    Position trans_kin(const Position& xt, const TranslationalVelocity& xt_dot, const TranslationalAcceleration& xt_ddot);
    OrientationQuaternion quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);
    AngularVelocity CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);
    AngularVelocity qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);
    EulerAngleRates wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);

    struct VerticalSpeed {
        double data;
    };

    struct SpecificForce {
        Eigen::Vector3d data;
    };

    struct Gravity {
        Eigen::Vector3d data;
    };

    struct CenterOfGravity {
        Eigen::Vector3d data;
    };

    struct Twist {
        TranslationalVelocity v;
        AngularVelocity w;
    };

    template <typename T>
    struct Twist_T {
        constants::Vector3_T<T> v = constants::Zero3_T<T>;
        constants::Vector3_T<T> w = constants::Zero3_T<T>;
    };

    template <typename T>
    struct Wrench_T {
        constants::Vector3_T<T> F = constants::Zero3_T<T>;
        constants::Vector3_T<T> M = constants::Zero3_T<T>;
    };

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    RigidBodyState compute_rigid_body_state(const frames::Frame& F);

    TranslationalVelocity trans_kin_vel(const TranslationalVelocity& xt_dot, const TranslationalAcceleration& xt_ddot);

    AngularVelocity eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);

    State_T<double> pack_rigid_body_state(const RigidBodyState& xN_t);

    StateVector_T<double> unpack_rigid_body_state(const RigidBodyState& xN_t);

    RigidBodyState step_rigid_body(const RigidBodyState& xB_BI_t, const Mass& mass, const InertiaTensor& J, const Wrench& WB_net_t);

}
#include "simulation/dynamics/public.tpp"
