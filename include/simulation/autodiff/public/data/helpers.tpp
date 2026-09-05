#pragma once
#include "simulation/autodiff/public/data/helpers.hpp"

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cstddef>
#include <stdexcept>

namespace autodiff
{

	template <typename T, std::size_t rows>
	constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(
		const CppAD::eigen_vector<T>& x)
	{
		if (x.size() != rows) {
			throw std::invalid_argument("autodiff::eigen_vector_from_cppad_vector: vector has incorrect size");
		}

		constants::MatrixX_T<T, rows, 1> out;
		for (std::size_t i = 0; i < rows; ++i) {
			out(static_cast<Eigen::Index>(i)) = x[i];
		}
		return out;
	}

	template <typename T, int rows>
	CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(
		const constants::MatrixX_T<T, rows, 1>& x)
	{
		CppAD::eigen_vector<T> out(static_cast<std::size_t>(rows));
		for (int i = 0; i < rows; ++i) {
			out[i] = x(static_cast<Eigen::Index>(i));
		}
		return out;
	}

	template <int rows>
	constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(
		const constants::MatrixX_T<double, rows, 1>& x)
	{
		return x.template cast<CppAD::AD<double>>();
	}

} // namespace autodiff
