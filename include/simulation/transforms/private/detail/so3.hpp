#pragma once
#include "simulation/transforms/public/data/types.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace transforms
{

	/**
	 * @brief Constructs an active rotation matrix about the x-axis.
	 *
	 * @param[in] phi Rotation angle [rad].
	 * @return Active x-axis rotation matrix [-].
	 */
	Eigen::Matrix3d Rx(double phi);

	/**
	 * @brief Constructs an active rotation matrix about the y-axis.
	 *
	 * @param[in] theta Rotation angle [rad].
	 * @return Active y-axis rotation matrix [-].
	 */
	Eigen::Matrix3d Ry(double theta);

	/**
	 * @brief Constructs an active rotation matrix about the z-axis.
	 *
	 * @param[in] psi Rotation angle [rad].
	 * @return Active z-axis rotation matrix [-].
	 */
	Eigen::Matrix3d Rz(double psi);

	/**
	 * @brief Constructs an active rotation matrix from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] roll First Euler angle in @p order [rad].
	 * @param[in] pitch Second Euler angle in @p order [rad].
	 * @param[in] yaw Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Active rotation matrix for the extrinsic sequence [-].
	 */
	Eigen::Matrix3d eul_to_R_extr(double roll, double pitch, double yaw, EulerOrder order);

	/**
	 * @brief Constructs the matrix counterpart to an intrinsic Euler-angle sequence.
	 *
	 * @param[in] roll First Euler angle in @p order [rad].
	 * @param[in] pitch Second Euler angle in @p order [rad].
	 * @param[in] yaw Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Rotation matrix counterpart for the intrinsic sequence [-].
	 */
	Eigen::Matrix3d eul_to_R_intr(double roll, double pitch, double yaw, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation matrix from an extrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Coordinate-transformation matrix for the extrinsic sequence [-].
	 */
	Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Constructs a coordinate-transformation matrix from an intrinsic Euler-angle sequence.
	 *
	 * @param[in] a First Euler angle in @p order [rad].
	 * @param[in] b Second Euler angle in @p order [rad].
	 * @param[in] c Third Euler angle in @p order [rad].
	 * @param[in] order Euler-axis sequence.
	 * @return Coordinate-transformation matrix for the intrinsic sequence [-].
	 */
	Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, EulerOrder order);

	/**
	 * @brief Extracts extrinsic Euler angles from an active rotation matrix.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, EulerOrder order);

	/**
	 * @brief Extracts intrinsic Euler angles from a rotation-matrix counterpart.
	 *
	 * @param[in] R Rotation matrix counterpart for an intrinsic sequence [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d R_to_eul_intr(const Eigen::Matrix3d& R, EulerOrder order);

	/**
	 * @brief Extracts extrinsic Euler angles from a coordinate-transformation matrix.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d C_to_eul_extr(const Eigen::Matrix3d& C, EulerOrder order);

	/**
	 * @brief Extracts intrinsic Euler angles from a coordinate-transformation matrix.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] order Euler-axis sequence.
	 * @return Euler angles ordered as the first, second, and third rotations [rad].
	 */
	Eigen::Vector3d C_to_eul_intr(const Eigen::Matrix3d& C, EulerOrder order);

	/**
	 * @brief Multiplies rotation matrices in list order.
	 *
	 * @param[in] rot_list Rotation matrices to multiply [-].
	 * @return Product of the rotation matrices in list order [-].
	 */
	Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list);

	/**
	 * @brief Multiplies rotation matrices in reverse list order.
	 *
	 * @param[in] rot_list Rotation matrices to multiply [-].
	 * @return Product of the rotation matrices in reverse list order [-].
	 */
	Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list);

} // namespace transforms
