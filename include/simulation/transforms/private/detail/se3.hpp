#pragma once
#include <Eigen/Dense>
#include <vector>

namespace transforms
{

	/**
	 * @brief Constructs an active homogeneous transformation that translates before rotating.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation that applies @p d before @p R.
	 */
	Eigen::Matrix4d make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs an active homogeneous transformation that rotates before translating.
	 *
	 * @param[in] R Active rotation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation that applies @p R before @p d.
	 */
	Eigen::Matrix4d make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs a passive homogeneous transformation that translates before rotating.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation with upper blocks C and -C d.
	 */
	Eigen::Matrix4d make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

	/**
	 * @brief Constructs a passive homogeneous transformation that rotates before translating.
	 *
	 * @param[in] C Coordinate-transformation matrix [-].
	 * @param[in] d Translation vector, in the same length units as transformed points.
	 * @return Homogeneous transformation with upper blocks C and -d.
	 */
	Eigen::Matrix4d make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

	/**
	 * @brief Multiplies homogeneous transformations in list order.
	 *
	 * @param[in] H_list Homogeneous transformations to multiply.
	 * @return Product of the transformations in list order.
	 */
	Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list);

	/**
	 * @brief Multiplies homogeneous transformations in reverse list order.
	 *
	 * @param[in] H_list Homogeneous transformations to multiply.
	 * @return Product of the transformations in reverse list order.
	 */
	Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list);

} // namespace transforms
