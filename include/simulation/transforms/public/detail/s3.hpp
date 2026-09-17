#pragma once
#include "simulation/transforms/public/data/types.hpp"

#include <Eigen/Dense>

namespace transforms
{

	/**
	 * @brief Constructs an active-rotation quaternion from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatR(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation quaternion from an Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @param[in] type Extrinsic or intrinsic frame-rotation sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatC(double a, double b, double c, EulerOrder order, RotationType type);

	/**
	 * @brief Extracts extrinsic Euler angles from an active-rotation quaternion.
	 *
	 * @param[in] qR Active-rotation quaternion [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatR_to_eul(const Eigen::Quaterniond& qR, EulerOrder order);

	/**
	 * @brief Extracts Euler angles from a coordinate-transformation quaternion.
	 *
	 * @param[in] qC Coordinate-transformation quaternion [-].
	 * @param[in] order Euler-axis sequence.
	 * @param[in] type Extrinsic or intrinsic frame-rotation sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, EulerOrder order, RotationType type);

	/**
	 * @brief Converts a rotation representation from quaternion form to matrix form.
	 *
	 * The input quaternion is normalized and canonicalized before conversion.
	 *
	 * @param[in] q Quaternion representing a rotation or coordinate transformation [-].
	 * @return Equivalent rotation or coordinate-transformation matrix [-].
	 */
	Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q);

	/**
	 * @brief Normalizes a quaternion and canonicalizes its sign.
	 *
	 * @param[in] q Quaternion to normalize and canonicalize [-].
	 * @return Unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q);
} // namespace transforms
