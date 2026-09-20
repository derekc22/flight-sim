#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstddef>

namespace util
{

	/**
	 * @brief Clamps a scalar to an inclusive interval.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] min_value Inclusive lower bound.
	 * @param[in] max_value Inclusive upper bound.
	 * @return @p x limited to the interval from @p min_value to @p max_value.
	 */
	double clamp(double x, double min_value, double max_value);

	/**
	 * @brief Clamps a scalar to symmetric bounds.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] max_abs Nonnegative bound magnitude.
	 * @return @p x limited to symmetric bounds of magnitude @p max_abs, or zero when the bound is nonpositive.
	 */
	double clamp_symmetric(double x, double max_abs);

	/**
	 * @brief Clamps a scalar to nonnegative bounds.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] max_value Nonnegative upper bound.
	 * @return @p x limited to [0, @p max_value], or zero when @p max_value is nonpositive.
	 */
	double clamp_positive(double x, double max_value);

	/**
	 * @brief Clamps a scalar to the interval from -1 to 1.
	 *
	 * @param[in] x Value to clamp.
	 * @return @p x limited to [-1, 1].
	 */
	double clamp_to_1(double x);

	/**
	 * @brief Clamps a scalar strictly inside the interval from -1 to 1.
	 *
	 * @param[in] x Value to clamp.
	 * @return @p x limited to [-1 + @c constants::eps, 1 - @c constants::eps].
	 */
	double clamp_inside_1(double x);

	/**
	 * @brief Wraps an angle to the principal remainder about zero.
	 *
	 * @param[in] x Angle to wrap [rad].
	 * @return Equivalent principal angle [rad].
	 */
	double wrap_to_pi(double x);

	double sqrt(double x);

	double abs(double x);

	/**
	 * @brief Computes a differentiable approximation of absolute value.
	 *
	 * @param[in] x Input value.
	 * @return Smooth approximation of the absolute value of @p x.
	 */
	double smooth_abs(double x);

	/**
	 * @brief Clamps each vector element between corresponding lower and upper bounds.
	 *
	 * @param[in] x Vector to clamp.
	 * @param[in] x_min Componentwise lower bounds.
	 * @param[in] x_max Componentwise upper bounds.
	 * @return Componentwise-clamped vector.
	 */
	Eigen::VectorXd vec_clamp(const Eigen::VectorXd& x, const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max);

	/**
	 * @brief Checks whether two vectors have equal sizes and approximately equal values.
	 *
	 * @param[in] a First vector.
	 * @param[in] b Second vector.
	 * @retval true If the vectors have equal sizes and are approximately equal.
	 * @retval false Otherwise.
	 */
	bool vec_is_close(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

	/**
	 * @brief Fills a half-open range of a fixed-size array with a value.
	 *
	 * @tparam T Array element type.
	 * @tparam N Number of elements in the array.
	 * @param[in,out] x Array to modify.
	 * @param[in] start Inclusive starting index.
	 * @param[in] stop Exclusive ending index.
	 * @param[in] val Value assigned to each selected element.
	 */
	template <typename T, std::size_t N>
	void fill_arr(std::array<T, N>& x, std::size_t start, std::size_t stop, const T& val);
} // namespace util

#include "simulation/util/public/math.tpp"
