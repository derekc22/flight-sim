#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/validation.hpp"

namespace util {

	template <typename T>
	void validate_shape(
	    const Eigen::DenseBase<T>& M,
	    Eigen::Index rows,
	    Eigen::Index cols,
	    const std::string& context,
	    const std::string& name)
	{
		if (M.rows() != rows || M.cols() != cols) {
			throw std::invalid_argument("util::validate_shape: " + context + ": incorrect shape for " + name +
			    ". Must be (" + std::to_string(rows) + " x " + std::to_string(cols) + ")" + ". Got (" +
			    std::to_string(M.rows()) + " x " + std::to_string(M.cols()) + ")");
		}
	}

	template <typename T>
	void validate_square(
	    const Eigen::DenseBase<T>& M,
	    const std::string& context,
	    const std::string& name)
	{
		if (M.rows() != M.cols()) {
			throw std::invalid_argument("util::validate_square: " + context + ": " + name + " must be square. Got (" +
			    std::to_string(M.rows()) + " x " + std::to_string(M.cols()) + ")");
		}
	}

	template <typename T>
	void validate_unit(
	    const Eigen::MatrixBase<T>& v,
	    const std::string& context,
	    const std::string& name)
	{
		const double n = v.norm();
		if (std::abs(n - 1.0) > constants::eps) {
			throw std::invalid_argument("util::validate_unit: " + context + ": " + name +
			    " must have unit norm. Got norm = " + std::to_string(n));
		}
	}

	template <typename T>
	void validate_value(
	    T a,
	    T b,
	    const std::string& context,
	    const std::string& name_a,
	    const std::string& name_b)
	{
		if (a != b) {
			throw std::invalid_argument("util::validate_value: " + context + ": " + name_a + " and " + name_b +
			    " must be equal. Got " + name_a + " = " + std::to_string(a) + ", " + name_b + " = " +
			    std::to_string(b));
		}
	}

} // namespace util
