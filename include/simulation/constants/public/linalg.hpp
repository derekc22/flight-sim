#pragma once
#include <Eigen/Dense>

namespace constants {

	// Identity quantities
	inline const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
	inline const Eigen::Matrix4d HI = Eigen::Matrix4d::Identity();       // Identity homogeneous transformation matrix
	inline const Eigen::Quaterniond qI = Eigen::Quaterniond::Identity(); // Identity quaternion

	// Zero quantities
	inline const Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();
	inline const Eigen::Quaterniond q0 = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
	inline const Eigen::Vector3d Zero3 = Eigen::Vector3d::Zero();

	// Special vectors
	inline const Eigen::Vector3d ei = Eigen::Vector3d(1, 0, 0);
	inline const Eigen::Vector3d ej = Eigen::Vector3d(0, 1, 0);
	inline const Eigen::Vector3d ek = Eigen::Vector3d(0, 0, 1);

	// Template alias vectors and matrices
	template <typename T, int rows, int cols> using MatrixX_T = Eigen::Matrix<T, rows, cols>;

	template <typename T> using Vector3_T = MatrixX_T<T, 3, 1>;

	template <typename T> inline const Vector3_T<T> Zero3_T = Vector3_T<T>::Zero();

	template <typename T> using Matrix3_T = MatrixX_T<T, 3, 3>;

	template <typename T> inline const Matrix3_T<T> Zero3x3_T = Matrix3_T<T>::Zero();

	template <typename T, int dim> inline const MatrixX_T<T, dim, dim> I_T = MatrixX_T<T, dim, dim>::Identity();

} // namespace constants
