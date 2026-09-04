#pragma once
#include "simulation/estimation/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace json
{

	estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(
		const nlohmann::json& estimator_json);

	estimation::LinearKalmanEstimator make_linear_kalman_estimator(estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json);
	estimation::ExtendedKalmanEstimator make_extended_kalman_estimator(estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json);

	estimation::EstimatorType map_estimator_type(const std::string& estimator_type_str);
	estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_json);

	void parse_linear_kalman_estimator(const nlohmann::json& estimator_json,
		std::optional<estimation::LinearKalmanEstimator>& estimator);
	void parse_extended_kalman_estimator(const nlohmann::json& estimator_json,
		std::optional<estimation::ExtendedKalmanEstimator>& estimator);

	estimation::EstimationManager parse_estimation_manager(const nlohmann::json& config, bool trim_flag);
} // namespace json
