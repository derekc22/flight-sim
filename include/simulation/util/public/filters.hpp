#pragma once
#include <Eigen/Dense>

namespace util
{

	/**
	 * @brief Applies an exact discrete first-order lag to a scalar value.
	 *
	 * @param[in] val Current input value.
	 * @param[in] prev_val Previous filtered value.
	 * @param[in] tau Lag time constant; a nonpositive value bypasses the filter [s].
	 * @param[in] dt Time step [s].
	 * @return Current filtered value.
	 */
	double first_order_lag(double val, double prev_val, double tau, double dt);

	/**
	 * @brief Applies an exact discrete first-order lag componentwise to a vector.
	 *
	 * @param[in] val Current input vector.
	 * @param[in] prev_val Previous filtered vector.
	 * @param[in] tau Lag time constant; a nonpositive value bypasses the filter [s].
	 * @param[in] dt Time step [s].
	 * @return Current filtered vector.
	 */
	Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau, double dt);

	/**
	 * @brief Applies an exact discrete first-order lag between two quaternions.
	 *
	 * Uses shortest-path spherical interpolation from @p prev_val toward @p val.
	 * A nonpositive time constant returns @p val unchanged.
	 *
	 * @param[in] val Current input quaternion [-].
	 * @param[in] prev_val Previous filtered quaternion [-].
	 * @param[in] tau Lag time constant [s].
	 * @param[in] dt Time step [s].
	 * @return Current filtered quaternion [-].
	 */
	Eigen::Quaterniond
	first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau, double dt);

} // namespace util
