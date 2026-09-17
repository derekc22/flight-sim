#pragma once
#include "simulation/transforms/public/data/types.hpp"

#include <Eigen/Dense>

namespace transforms
{

	/**
	 * @brief Extracts the coordinate-transformation matrix from a homogeneous transformation.
	 *
	 * @param[in] H Homogeneous transformation matrix.
	 * @return Upper-left 3-by-3 coordinate-transformation block of @p H [-].
	 */
	Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H);

	/**
	 * @brief Extracts the active rotation matrix from a homogeneous transformation.
	 *
	 * @param[in] H Homogeneous transformation matrix.
	 * @return Upper-left 3-by-3 active rotation block of @p H [-].
	 */
	Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H);

	/**
	 * @brief Extracts the translation block from a homogeneous transformation.
	 *
	 * @param[in] H Homogeneous transformation matrix.
	 * @return Upper-right translation block of @p H, in the transformation's length units.
	 */
	Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H);

	/**
	 * @brief Extracts the frame-position vector from a passive homogeneous transformation.
	 *
	 * For a transformation with rotation block C and translation block -C p, this function returns p.
	 *
	 * @param[in] H Passive homogeneous transformation matrix.
	 * @return Frame-position vector, in the transformation's length units.
	 */
	Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H);

	/**
	 * @brief Constructs an active homogeneous transformation.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @param[in] order Order in which rotation and translation are applied.
	 * @return Homogeneous transformation constructed from @p R and @p d.
	 */
	Eigen::Matrix4d make_HR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, TransformationOrder order);

	/**
	 * @brief Constructs a passive homogeneous transformation.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @param[in] order Order in which rotation and translation are applied.
	 * @return Homogeneous transformation constructed from @p C and @p d.
	 */
	Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, TransformationOrder order);

	/**
	 * @brief Inverts a rigid homogeneous transformation.
	 *
	 * @param[in] H Homogeneous transformation matrix.
	 * @return Inverse of @p H.
	 */
	Eigen::Matrix4d make_Hinv(const Eigen::Matrix4d& H);

	/**
	 * @brief Applies a homogeneous transformation to a point.
	 *
	 * @param[in] H Homogeneous transformation matrix.
	 * @param[in] v Point to transform, in the same length units as the translation in @p H.
	 * @return Transformed point in the same length units as @p v.
	 */
	Eigen::Vector3d apply_H(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);

} // namespace transforms
