#pragma once

#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

namespace util
{

	CppAD::AD<double> csc(CppAD::AD<double> x);

	CppAD::AD<double> sec(CppAD::AD<double> x);

	CppAD::AD<double> cot(CppAD::AD<double> x);

	CppAD::AD<double> sin(const CppAD::AD<double>& x);

	CppAD::AD<double> cos(const CppAD::AD<double>& x);

	CppAD::AD<double> tan(const CppAD::AD<double>& x);

	CppAD::AD<double> asin(const CppAD::AD<double>& x);

	CppAD::AD<double> acos(const CppAD::AD<double>& x);

	CppAD::AD<double> atan2(const CppAD::AD<double>& y, const CppAD::AD<double>& x);

	CppAD::AD<double> sqrt(const CppAD::AD<double>& x);

	CppAD::AD<double> abs(const CppAD::AD<double>& x);

	/**
	 * @brief Computes a differentiable approximation of absolute value.
	 *
	 * @param[in] x Input value.
	 * @return Smooth approximation of the absolute value of @p x.
	 */
	CppAD::AD<double> smooth_abs(const CppAD::AD<double>& x);

	/**
	 * @brief Clamps an automatic-differentiation value to an inclusive interval.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] min_value Inclusive lower bound.
	 * @param[in] max_value Inclusive upper bound.
	 * @return @p x limited to the interval from @p min_value to @p max_value.
	 */
	CppAD::AD<double> clamp(const CppAD::AD<double>& x, double min_value, double max_value);

	/**
	 * @brief Clamps an automatic-differentiation value to symmetric bounds.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] max_abs Nonnegative bound magnitude.
	 * @return @p x limited to symmetric bounds of magnitude @p max_abs, or zero when the bound is nonpositive.
	 */
	CppAD::AD<double> clamp_symmetric(const CppAD::AD<double>& x, double max_abs);

	/**
	 * @brief Clamps an automatic-differentiation value to nonnegative bounds.
	 *
	 * @param[in] x Value to clamp.
	 * @param[in] max_value Nonnegative upper bound.
	 * @return @p x limited to [0, @p max_value], or zero when @p max_value is nonpositive.
	 */
	CppAD::AD<double> clamp_positive(const CppAD::AD<double>& x, double max_value);

	/**
	 * @brief Clamps an automatic-differentiation value to the interval from -1 to 1.
	 *
	 * @param[in] x Value to clamp.
	 * @return @p x limited to [-1, 1].
	 */
	CppAD::AD<double> clamp_to_1(const CppAD::AD<double>& x);

	/**
	 * @brief Returns the normalized direction of an automatic-differentiation vector.
	 *
	 * @param[in] v Three-dimensional vector to normalize.
	 * @return Unit vector parallel to @p v, or the zero vector when its norm is below @c constants::eps.
	 */
	constants::MatrixX_T<CppAD::AD<double>, 3, 1> norm(const constants::MatrixX_T<CppAD::AD<double>, 3, 1>& v);

} // namespace util
