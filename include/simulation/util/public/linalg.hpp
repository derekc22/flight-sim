#pragma once

#include <Eigen/Dense>

namespace util
{

	/**
	 * @brief Constructs the skew-symmetric cross-product matrix of a vector.
	 *
	 * The returned matrix satisfies hat(v) * x = v cross x.
	 *
	 * @param[in] v Vector to map into the Lie algebra so(3).
	 * @return Skew-symmetric matrix associated with @p v.
	 */
	Eigen::Matrix3d hat(const Eigen::Vector3d& v);

	/**
	 * @brief Extracts a vector from the independent entries of a skew-symmetric matrix.
	 *
	 * @param[in] S Matrix whose entries S(2,1), S(0,2), and S(1,0) are extracted.
	 * @return Vector formed from the selected lower- and upper-triangular entries.
	 */
	Eigen::Vector3d vee(const Eigen::Matrix3d& S);

	/**
	 * @brief Returns the normalized direction of a three-dimensional vector.
	 *
	 * @param[in] v Vector to normalize.
	 * @return Unit vector parallel to @p v, or the zero vector when its norm is below @c constants::eps.
	 */
	Eigen::Vector3d norm(const Eigen::Vector3d& v);

} // namespace util
