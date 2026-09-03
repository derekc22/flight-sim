#pragma once
#include <Eigen/Dense>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"

namespace dynamics {
	struct OrientationQuaternion;
	struct EulerAngles;
	struct OrientationQuaternionRate;
} // namespace dynamics

namespace dynamics {

	struct Position {
		Eigen::Vector3d data; // e.g. pI_BI [m]
	};

	struct OrientationMatrix {
		Eigen::Matrix3d data; // e.g. CIB [-]
		void set(const OrientationQuaternion& q);
		void set(const EulerAngles& eul);
	};

	struct HomogeneousTransformationMatrix {
		Eigen::Matrix4d data; // e.g. HIB
		OrientationMatrix C() const;
		Position p() const;
		void set(const OrientationMatrix& C, const Position& p);
		void set(const OrientationMatrix& C);
		void set(const Position& p);
		void set(const OrientationQuaternion& q);
		void set(const EulerAngles& eul);
	};

	struct OrientationQuaternion {
		Eigen::Quaterniond data; // e.g. qIB [-]
		void set(const OrientationMatrix& C);
		void set(const EulerAngles& eul);
	};

	struct EulerAngles {
		// "ZYX", intrinsic. Stored as [yaw, pitch, roll]
		Eigen::Vector3d data; // e.g. eulIB [rad]
		double psi() const;   // yaw
		double theta() const; // pitch
		double phi() const;   // roll
		void set(const OrientationMatrix& C);
		void set(const OrientationQuaternion& q);
	};

	struct AngularVelocity {
		Eigen::Vector3d data; // e.g. wB_BI [rad/s]
		double p() const;
		double q() const;
		double r() const;
	};

	struct OrientationMatrixRate {
		Eigen::Matrix3d data; // e.g. CIB_dot [s^-1]
		void set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C);
		void set(const OrientationMatrix& C, const AngularVelocity& w);
	};

	struct OrientationQuaternionRate {
		Eigen::Quaterniond data; // e.g. qIB_dot [s^-1]
		void set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q);
		void set(const OrientationQuaternion& q, const AngularVelocity& w);
	};

	struct EulerAngleRates {
		// "ZYX", intrinsic. Stored as [phi_dot, theta_dot, psi_dot]
		Eigen::Vector3d data; // e.g. eulIB_dot [rad/s]
		double phi_dot() const;
		double theta_dot() const;
		double psi_dot() const;
		void set(const AngularVelocity& w, const EulerAngles& eul);
	};

	struct AngularVelocityQuaternion {
		Eigen::Quaterniond data; // e.g. wq_BI = [ 0; -wB_BI ]. minus for qIB convention
		AngularVelocity w() const;
		void set(const AngularVelocity& w);
	};

	struct TranslationalVelocity {
		Eigen::Vector3d data; // e.g. vB_BI [m/s]
	};

	struct TranslationalAcceleration {
		Eigen::Vector3d data; // e.g. vB_BI_dot, aI_BI, or fB [ms^-2]
	};

	struct AngularAcceleration {
		Eigen::Vector3d data; // e.g. wB_BI_dot [rad/s^2]
	};

	struct Force {
		Eigen::Vector3d data; // e.g. FB [N]
	};

	struct InertiaTensor {
		Eigen::Matrix3d data; // e.g. JB [kg⋅m^2]
	};

	struct Mass {
		double data; // m [kg]
	};

	struct Moment {
		Eigen::Vector3d data; // e.g. MB [Nm]
	};

	struct RigidBodyState {
		Position p;              // e.g. pI_BI
		TranslationalVelocity v; // e.g. vB_BI
		OrientationQuaternion q; // e.g. qIB
		AngularVelocity w;       // e.g. wB_BI
	};

	struct RigidBodyStateDot {
		TranslationalVelocity p_dot;
		TranslationalAcceleration v_dot;
		AngularAcceleration w_dot;
	};

	struct Wrench {
		Force F;  // e.g. FB [N]
		Moment M; // e.g. MB [Nm]
	};

	struct VerticalSpeed {
		double data; // e.g. alt_BE_dot [m/s]
	};

	struct Gravity {
		Eigen::Vector3d data; // e.g. gB [ms^-2]
	};

	struct WrenchSet {
		Wrench aerodynamic;
		Wrench propulsive;
		Wrench net;
	};

	template <typename T> using StateVector_T = constants::MatrixX_T<T, constants::state_dim, 1>;

	template <typename T> using StateDotVector_T = constants::MatrixX_T<T, constants::state_dim, 1>;

	template <typename T> using WrenchVector_T = constants::MatrixX_T<T, constants::virtual_input_dim, 1>;

	template <typename T> struct State_T {
		T vx = T(0.0);
		T vy = T(0.0);
		T vz = T(0.0);

		T p = T(0.0);
		T q = T(0.0);
		T r = T(0.0);

		T phi = T(0.0);
		T theta = T(0.0);
	};

	template <typename T> struct StateDot_T {
		T vx_dot = T(0.0);
		T vy_dot = T(0.0);
		T vz_dot = T(0.0);

		T p_dot = T(0.0);
		T q_dot = T(0.0);
		T r_dot = T(0.0);

		T phi_dot = T(0.0);
		T theta_dot = T(0.0);
	};

	template <typename T> struct Twist_T {
		constants::Vector3_T<T> v = constants::Zero3_T<T>;
		constants::Vector3_T<T> w = constants::Zero3_T<T>;
	};

	template <typename T> struct Wrench_T {
		constants::Vector3_T<T> F = constants::Zero3_T<T>;
		constants::Vector3_T<T> M = constants::Zero3_T<T>;
	};

	template <typename T> struct WrenchSet_T {
		Wrench_T<T> aerodynamic;
		Wrench_T<T> propulsive;
		Wrench_T<T> net;
	};

} // namespace dynamics
