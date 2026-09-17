#pragma once
#include <Eigen/Dense>
#include <vector>

namespace transforms
{

	/**
	 * @brief Constructs an active homogeneous transformation that translates before rotating.
	 *
	 * The returned matrix has rotation block R and translation block R d.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation that applies @p d before @p R.
	 */
	Eigen::Matrix4d make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs an active homogeneous transformation that rotates before translating.
	 *
	 * The returned matrix has rotation block R and translation block d.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation that applies @p R before @p d.
	 */
	Eigen::Matrix4d make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs a passive homogeneous transformation that translates before rotating.
	 *
	 * The returned matrix has coordinate-transformation block C and translation block -C d.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation with upper blocks C and -C d.
	 */
	Eigen::Matrix4d make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs a passive homogeneous transformation that rotates before translating.
	 *
	 * The returned matrix has coordinate-transformation block C and translation block -d.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation with upper blocks C and -d.
	 */
	Eigen::Matrix4d make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

	/**
	 * @brief Multiplies homogeneous transformations in list order.
	 *
	 * For a list containing H1, H2, and H3, the result is H1 H2 H3. An empty list returns the identity
	 * transformation.
	 *
	 * @param[in] H_list Homogeneous transformations to multiply.
	 * @return Product of the transformations in list order.
	 */
	Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list);

	/**
	 * @brief Multiplies homogeneous transformations in reverse list order.
	 *
	 * For a list containing H1, H2, and H3, the result is H3 H2 H1. An empty list returns the identity
	 * transformation.
	 *
	 * @param[in] H_list Homogeneous transformations to multiply.
	 * @return Product of the transformations in reverse list order.
	 */
	Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list);

} // namespace transforms
