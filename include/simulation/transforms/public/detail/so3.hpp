#pragma once
#include "simulation/transforms/public/data/types.hpp"

#include <Eigen/Dense>

namespace transforms
{

	/**
	 * @brief Converts an active rotation matrix to a coordinate-transformation matrix.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @return Coordinate-transformation matrix given by the transpose of @p R [-].
	 */
	Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R);

	/**
	 * @brief Converts a coordinate-transformation matrix to an active rotation matrix.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @return Active rotation matrix given by the transpose of @p C [-].
	 */
	Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C);

	/**
	 * @brief Constructs an active rotation matrix from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Active rotation matrix for the Euler-angle sequence [-].
	 */
	Eigen::Matrix3d eul_to_R(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation matrix from an Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @param[in] type Extrinsic or intrinsic frame-rotation sequence.
	 * @return Coordinate-transformation matrix for the Euler-angle sequence [-].
	 */
	Eigen::Matrix3d eul_to_C(double a, double b, double c, EulerOrder order, RotationType type);

	/**
	 * @brief Extracts extrinsic Euler angles from an active rotation matrix.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d R_to_eul(const Eigen::Matrix3d& R, EulerOrder order);

	/**
	 * @brief Extracts Euler angles from a coordinate-transformation matrix.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] order Euler-axis sequence.
	 * @param[in] type Extrinsic or intrinsic frame-rotation sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, EulerOrder order, RotationType type);

	/**
	 * @brief Converts a rotation representation from matrix form to quaternion form.
	 *
	 * The returned quaternion is normalized and canonicalized to have a nonnegative scalar coefficient.
	 *
	 * @param[in] rot Rotation or coordinate-transformation matrix [-].
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);
} // namespace transforms
