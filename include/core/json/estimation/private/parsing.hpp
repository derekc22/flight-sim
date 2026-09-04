#pragma once
#include "simulation/estimation/public/manager.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace json
{

	estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(
		const nlohmann::json& estimator_json);

	estimation::LinearKalmanFilter make_linear_kalman_filter(estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json);
	estimation::ExtendedKalmanFilter make_extended_kalman_filter(estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json);

	estimation::EstimatorType map_estimator_type(const std::string& estimator_type_str);
	estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_json);

	void parse_linear_kalman_filter(const nlohmann::json& estimator_json,
		std::optional<estimation::LinearKalmanFilter>& estimator);
	void parse_extended_kalman_filter(const nlohmann::json& estimator_json,
		std::optional<estimation::ExtendedKalmanFilter>& estimator);

	estimation::EstimationManager parse_estimation_manager(const nlohmann::json& config, bool trim_flag);
} // namespace json
