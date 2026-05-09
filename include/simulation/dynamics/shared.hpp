#pragma once
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"

namespace dynamics {

    struct OrientationQuaternion; // forward declare
    struct EulerAngles; // forward declare
    struct OrientationQuaternionRate; // forward declare

    struct Position {
        Eigen::Vector3d data;
    };

    struct OrientationMatrix {
        Eigen::Matrix3d data;
        void set(const OrientationQuaternion& q);
        void set(const EulerAngles& eul);
    };

    struct HomogenousFrameTransformationMatrix {
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

    struct LinearVelocity {
        Eigen::Vector3d data;
    };

    struct LinearAcceleration {
        Eigen::Vector3d data;
    };

    struct AngularAcceleration {
        Eigen::Vector3d data;
    };

    struct VerticalSpeed {
        double data;
    };

    struct Force {
        Eigen::Vector3d data;
    };

    struct SpecificForce {
        Eigen::Vector3d data;
    };

    struct Gravity {
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

    struct CenterOfGravity {
        Eigen::Vector3d data;
    };

    struct RigidBodyState {
        Position p;
        LinearVelocity v;
        OrientationQuaternion q;
        AngularVelocity w;
    };

    struct Wrench {
        Force F;
        Moment M;
    };

    struct Twist {
        LinearVelocity v;
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
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot);

}

#include "simulation/dynamics/shared.tpp"
