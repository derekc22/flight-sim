#pragma once
#include "simulation/transforms/public/data/types.hpp"

#include <Eigen/Dense>
#include <vector>

namespace transforms
{

	/** @deprecated */
	// Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

	/**
	 * @brief Constructs an active-rotation quaternion about the x-axis.
	 *
	 * @param[in] phi Rotation angle [rad].
	 * @return Unit x-axis rotation quaternion [-].
	 */
	Eigen::Quaterniond qx(double phi);

	/**
	 * @brief Constructs an active-rotation quaternion about the y-axis.
	 *
	 * @param[in] theta Rotation angle [rad].
	 * @return Unit y-axis rotation quaternion [-].
	 */
	Eigen::Quaterniond qy(double theta);

	/**
	 * @brief Constructs an active-rotation quaternion about the z-axis.
	 *
	 * @param[in] psi Rotation angle [rad].
	 * @return Unit z-axis rotation quaternion [-].
	 */
	Eigen::Quaterniond qz(double psi);

	/**
	 * @brief Constructs an active-rotation quaternion from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatR_extr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs the quaternion counterpart to an intrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatR_intr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation quaternion from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatC_extr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation quaternion from an intrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Equivalent unit quaternion with a nonnegative scalar coefficient [-].
	 */
	Eigen::Quaterniond eul_to_quatC_intr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Extracts extrinsic Euler angles from an active-rotation quaternion.
	 *
	 * @param[in] q Active-rotation quaternion [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatR_to_eul_extr(const Eigen::Quaterniond& q, EulerOrder order);

	/**
	 * @brief Extracts intrinsic Euler angles from a rotation-quaternion counterpart.
	 *
	 * @param[in] q Rotation-quaternion counterpart for an intrinsic sequence [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatR_to_eul_intr(const Eigen::Quaterniond& q, EulerOrder order);

	/**
	 * @brief Extracts extrinsic Euler angles from a coordinate-transformation quaternion.
	 *
	 * @param[in] q Coordinate-transformation quaternion [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatC_to_eul_extr(const Eigen::Quaterniond& q, EulerOrder order);

	/**
	 * @brief Extracts intrinsic Euler angles from a coordinate-transformation quaternion.
	 *
	 * @param[in] q Coordinate-transformation quaternion [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d quatC_to_eul_intr(const Eigen::Quaterniond& q, EulerOrder order);

	/**
	 * @brief Multiplies quaternions in list order.
	 *
	 * For a list containing q1, q2, and q3, the result is q1 q2 q3. Each quaternion is normalized and
	 * canonicalized before multiplication, and the result is canonicalized. An empty list returns the identity
	 * quaternion.
	 *
	 * @param[in] q_list Quaternions to multiply [-].
	 * @return Product of the quaternions in list order [-].
	 */
	Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list);

	/**
	 * @brief Multiplies quaternions in reverse list order.
	 *
	 * For a list containing q1, q2, and q3, the result is q3 q2 q1. Each quaternion is normalized and
	 * canonicalized before multiplication, and the result is canonicalized. An empty list returns the identity
	 * quaternion.
	 *
	 * @param[in] q_list Quaternions to multiply [-].
	 * @return Product of the quaternions in reverse list order [-].
	 */
	Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list);

} // namespace transforms
