#include <stdexcept>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/public/data/helpers.hpp"

namespace json {

	Eigen::Vector3d parse_Vector3d(
	    const nlohmann::json& values)
	{
		if (!values.is_array() || values.size() != 3) {
			throw std::runtime_error("expected a 3-element array");
		}
		return {values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>()};
	}

	Eigen::Vector4d parse_Vector4d(
	    const nlohmann::json& values)
	{
		if (!values.is_array() || values.size() != 4) {
			throw std::runtime_error("expected a 4-element array");
		}
		return {values.at(0).get<double>(),
		    values.at(1).get<double>(),
		    values.at(2).get<double>(),
		    values.at(3).get<double>()};
	}

	Eigen::Matrix3d parse_Matrix3d(
	    const nlohmann::json& values)
	{
		if (!values.is_array() || values.size() != 3) {
			throw std::runtime_error("expected a 3x3 array");
		}

		Eigen::Matrix3d out;
		for (int row = 0; row < 3; ++row) {
			const auto& row_values = values.at(row);
			if (!row_values.is_array() || row_values.size() != 3) {
				throw std::runtime_error("expected a 3x3 array");
			}
			for (int col = 0; col < 3; ++col) {
				out(row, col) = row_values.at(col).get<double>();
			}
		}
		return out;
	}

	Eigen::Matrix4d parse_Matrix4d(
	    const nlohmann::json& values)
	{
		if (!values.is_array() || values.size() != 4) {
			throw std::runtime_error("expected a 4x4 array");
		}

		Eigen::Matrix4d out;
		for (int row = 0; row < 4; ++row) {
			const auto& row_values = values.at(row);
			if (!row_values.is_array() || row_values.size() != 4) {
				throw std::runtime_error("expected a 4x4 array");
			}
			for (int col = 0; col < 4; ++col) {
				out(row, col) = row_values.at(col).get<double>();
			}
		}
		return out;
	}

	Eigen::MatrixXd parse_MatrixXd(
	    const nlohmann::json& values)
	{
		if (!values.is_array()) {
			throw std::runtime_error("expected matrix array");
		}
		if (values.empty()) {
			return Eigen::MatrixXd{};
		}

		const int rows = static_cast<int>(values.size());
		const int cols = static_cast<int>(values.at(0).size());
		Eigen::MatrixXd out(rows, cols);

		for (int row = 0; row < rows; ++row) {
			const auto& row_values = values.at(row);
			if (!row_values.is_array() || static_cast<int>(row_values.size()) != cols) {
				throw std::runtime_error("expected rectangular matrix");
			}
			for (int col = 0; col < cols; ++col) {
				out(row, col) = row_values.at(col).get<double>();
			}
		}
		return out;
	}

	Eigen::Quaterniond parse_Quaterniond(
	    const nlohmann::json& values)
	{
		const Eigen::Vector4d q = parse_Vector4d(values);
		return {q(0), q(1), q(2), q(3)};
	}

} // namespace json
