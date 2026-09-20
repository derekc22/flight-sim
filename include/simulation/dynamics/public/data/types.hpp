#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>

namespace dynamics
{
	struct OrientationQuaternion;
	struct EulerAngles;
	struct OrientationQuaternionRate;
} // namespace dynamics

namespace dynamics
{

	struct Position {
		Eigen::Vector3d data; // e.g. pI_BI [m]
	};

	struct OrientationMatrix {
		Eigen::Matrix3d data; // e.g. CIB [-]

		/**
		 * @brief Sets the orientation matrix from an orientation quaternion.
		 *
		 * @param[in] q Orientation quaternion [-].
		 */
		void set(const OrientationQuaternion& q);

		/**
		 * @brief Sets the orientation matrix from intrinsic ZYX Euler angles.
		 *
		 * @param[in] eul Euler angles [rad].
		 */
		void set(const EulerAngles& eul);
	};

	struct HomogeneousTransformationMatrix {
		Eigen::Matrix4d data; // e.g. HIB

		/**
		 * @brief Extracts the orientation matrix from the homogeneous transformation.
		 *
		 * @return Orientation matrix in the upper-left block [-].
		 */
		OrientationMatrix C() const;

		/**
		 * @brief Extracts the frame-position vector from the homogeneous transformation.
		 *
		 * @return Frame-position vector [m].
		 */
		Position p() const;

		/**
		 * @brief Sets the orientation and position represented by the homogeneous transformation.
		 *
		 * @param[in] C Orientation matrix [-].
		 * @param[in] p Frame-position vector [m].
		 */
		void set(const OrientationMatrix& C, const Position& p);

		/**
		 * @brief Sets the orientation matrix while preserving the current position.
		 *
		 * @param[in] C Orientation matrix [-].
		 */
		void set(const OrientationMatrix& C);

		/**
		 * @brief Sets the position while preserving the current orientation.
		 *
		 * @param[in] p Frame-position vector [m].
		 */
		void set(const Position& p);

		/**
		 * @brief Sets the orientation from a quaternion while preserving the current position.
		 *
		 * @param[in] q Orientation quaternion [-].
		 */
		void set(const OrientationQuaternion& q);

		/**
		 * @brief Sets the orientation from intrinsic ZYX Euler angles while preserving the current position.
		 *
		 * @param[in] eul Euler angles [rad].
		 */
		void set(const EulerAngles& eul);
	};

	struct OrientationQuaternion {
		Eigen::Quaterniond data; // e.g. qIB [-]

		/**
		 * @brief Sets the orientation quaternion from an orientation matrix.
		 *
		 * @param[in] C Orientation matrix [-].
		 */
		void set(const OrientationMatrix& C);

		/**
		 * @brief Sets the orientation quaternion from intrinsic ZYX Euler angles.
		 *
		 * @param[in] eul Euler angles [rad].
		 */
		void set(const EulerAngles& eul);
	};

	struct EulerAngles {
		// "ZYX", intrinsic. Stored as [yaw, pitch, roll]
		Eigen::Vector3d data; // e.g. eulIB [rad]

		/**
		 * @brief Returns the yaw angle.
		 *
		 * @return Yaw angle [rad].
		 */
		double psi() const; // yaw

		/**
		 * @brief Returns the pitch angle.
		 *
		 * @return Pitch angle [rad].
		 */
		double theta() const; // pitch

		/**
		 * @brief Returns the roll angle.
		 *
		 * @return Roll angle [rad].
		 */
		double phi() const; // roll

		/**
		 * @brief Sets intrinsic ZYX Euler angles from an orientation matrix.
		 *
		 * @param[in] C Orientation matrix [-].
		 */
		void set(const OrientationMatrix& C);

		/**
		 * @brief Sets intrinsic ZYX Euler angles from an orientation quaternion.
		 *
		 * @param[in] q Orientation quaternion [-].
		 */
		void set(const OrientationQuaternion& q);
	};

	struct AngularVelocity {
		Eigen::Vector3d data; // e.g. wB_BI [rad/s]

		/**
		 * @brief Returns the angular-velocity component about the body x-axis.
		 *
		 * @return Roll-rate component [rad/s].
		 */
		double p() const;

		/**
		 * @brief Returns the angular-velocity component about the body y-axis.
		 *
		 * @return Pitch-rate component [rad/s].
		 */
		double q() const;

		/**
		 * @brief Returns the angular-velocity component about the body z-axis.
		 *
		 * @return Yaw-rate component [rad/s].
		 */
		double r() const;
	};

	struct OrientationMatrixRate {
		Eigen::Matrix3d data; // e.g. CIB_dot [s^-1]

		/**
		 * @brief Sets the orientation-matrix rate from quaternion orientation and rate.
		 *
		 * @param[in] q_dot Orientation-quaternion rate [1/s].
		 * @param[in] q Orientation quaternion [-].
		 * @param[in] C Orientation matrix [-].
		 */
		void set(const OrientationQuaternionRate& q_dot, const OrientationQuaternion& q, const OrientationMatrix& C);

		/**
		 * @brief Sets the orientation-matrix rate from orientation and angular velocity.
		 *
		 * @param[in] C Orientation matrix [-].
		 * @param[in] w Angular velocity [rad/s].
		 */
		void set(const OrientationMatrix& C, const AngularVelocity& w);
	};

	struct OrientationQuaternionRate {
		Eigen::Quaterniond data; // e.g. qIB_dot [s^-1]

		/**
		 * @brief Sets the orientation-quaternion rate from matrix orientation and rate.
		 *
		 * @param[in] C_dot Orientation-matrix rate [1/s].
		 * @param[in] C Orientation matrix [-].
		 * @param[in] q Orientation quaternion [-].
		 */
		void set(const OrientationMatrixRate& C_dot, const OrientationMatrix& C, const OrientationQuaternion& q);

		/**
		 * @brief Sets the orientation-quaternion rate from orientation and angular velocity.
		 *
		 * @param[in] q Orientation quaternion [-].
		 * @param[in] w Angular velocity [rad/s].
		 */
		void set(const OrientationQuaternion& q, const AngularVelocity& w);
	};

	struct EulerAngleRates {
		// "ZYX", intrinsic. Stored as [phi_dot, theta_dot, psi_dot]
		Eigen::Vector3d data; // e.g. eulIB_dot [rad/s]

		/**
		 * @brief Returns the roll-angle rate.
		 *
		 * @return Roll-angle rate [rad/s].
		 */
		double phi_dot() const;

		/**
		 * @brief Returns the pitch-angle rate.
		 *
		 * @return Pitch-angle rate [rad/s].
		 */
		double theta_dot() const;

		/**
		 * @brief Returns the yaw-angle rate.
		 *
		 * @return Yaw-angle rate [rad/s].
		 */
		double psi_dot() const;

		/**
		 * @brief Sets intrinsic ZYX Euler-angle rates from angular velocity and orientation.
		 *
		 * @param[in] w Angular velocity [rad/s].
		 * @param[in] eul Intrinsic ZYX Euler angles [rad].
		 */
		void set(const AngularVelocity& w, const EulerAngles& eul);
	};

	struct AngularVelocityQuaternion {
		Eigen::Quaterniond data; // e.g. wq_BI = [ 0; wB_BI ]

		/**
		 * @brief Extracts angular velocity from the quaternion vector part.
		 *
		 * @return Angular velocity [rad/s].
		 */
		AngularVelocity w() const;

		/**
		 * @brief Sets a pure quaternion from angular velocity.
		 *
		 * @param[in] w Angular velocity [rad/s].
		 */
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
		Position p;				 // e.g. pI_BI
		TranslationalVelocity v; // e.g. vB_BI
		OrientationQuaternion q; // e.g. qIB
		AngularVelocity w;		 // e.g. wB_BI
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

	template <typename T> using StateVector_T = constants::MatrixX_T<T, constants::nx, 1>;

	template <typename T> using StateDotVector_T = constants::MatrixX_T<T, constants::nx, 1>;

	template <typename T> using WrenchVector_T = constants::MatrixX_T<T, constants::nv, 1>;

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
