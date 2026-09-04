#include "core/json/estimation/private/parsing.hpp"

#include "core/json/estimation/private/validation.hpp"
#include "core/json/public/data/helpers.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/estimation/public/manager.hpp"
#include "simulation/util/public/validation.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>

namespace json
{

	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> parse_kalman_filter_parameters(
		const nlohmann::json& estimator_json)
	{
		const auto& parameters_json = estimator_json.at("parameters");
		if (!parameters_json.is_object()) {
			throw std::runtime_error("json::parse_kalman_filter_parameters expected parameters object");
		}
		if (!parameters_json.contains("P0") || !parameters_json.contains("Q") || !parameters_json.contains("R")) {
			throw std::runtime_error("json::parse_kalman_filter_parameters requires P0, Q, and R");
		}

		Eigen::MatrixXd P0 = parse_MatrixXd(parameters_json.at("P0"));
		Eigen::MatrixXd Q = parse_MatrixXd(parameters_json.at("Q"));
		Eigen::MatrixXd R = parse_MatrixXd(parameters_json.at("R"));

		const std::string context = "json::parse_kalman_filter_parameters";
		util::validate_shape(P0, constants::state_dim, constants::state_dim, context, "P0");
		util::validate_shape(Q, constants::state_dim, constants::state_dim, context, "Q");
		util::validate_shape(R, constants::state_dim, constants::state_dim, context, "R");

		return {P0, Q, R};
	}

	estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(
		const nlohmann::json& estimator_json)
	{
		auto [P0, Q, R] = parse_kalman_filter_parameters(estimator_json);
		return {P0, Q, R};
	}

	estimation::ExtendedKalmanFilterParameters parse_extended_kalman_filter_parameters(
		const nlohmann::json& estimator_json)
	{
		auto [P0, Q, R] = parse_kalman_filter_parameters(estimator_json);
		return {P0, Q, R};
	}

	estimation::LinearKalmanEstimator make_linear_kalman_estimator(
		estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json)
	{
		switch (estimator_type) {
			case estimation::EstimatorType::LinearKalmanFilter: {
				estimation::LinearKalmanFilterParameters params = parse_linear_kalman_filter_parameters(estimator_json);
				return estimation::LinearKalmanEstimator(params);
			}

			default:
				throw std::runtime_error("json::make_linear_kalman_estimator unknown estimator type");
		}
	}

	estimation::ExtendedKalmanEstimator make_extended_kalman_estimator(
		estimation::EstimatorType estimator_type,
		const nlohmann::json& estimator_json)
	{
		switch (estimator_type) {
			case estimation::EstimatorType::ExtendedKalmanFilter: {
				estimation::ExtendedKalmanFilterParameters params =
					parse_extended_kalman_filter_parameters(estimator_json);
				return estimation::ExtendedKalmanEstimator(params);
			}

			default:
				throw std::runtime_error("json::make_extended_kalman_estimator unknown estimator type");
		}
	}

	estimation::EstimatorType map_estimator_type(
		const std::string& estimator_type_str)
	{
		if (estimator_type_str == "None") {
			return estimation::EstimatorType::None;
		}
		if (estimator_type_str == "LinearKalmanFilter") {
			return estimation::EstimatorType::LinearKalmanFilter;
		}
		if (estimator_type_str == "ExtendedKalmanFilter") {
			return estimation::EstimatorType::ExtendedKalmanFilter;
		}
		throw std::runtime_error("json::map_estimator_type unknown estimator type: " + estimator_type_str);
	}

	estimation::EstimatorType fetch_estimator_type(
		const nlohmann::json& estimator_json)
	{
		std::string estimator_type_str = estimator_json.at("estimator_type").get<std::string>();
		return map_estimator_type(estimator_type_str);
	}

	void parse_linear_kalman_estimator(
		const nlohmann::json& estimator_json,
		std::optional<estimation::LinearKalmanEstimator>& estimator)
	{
		estimator = make_linear_kalman_estimator(fetch_estimator_type(estimator_json), estimator_json);
	}

	void parse_extended_kalman_estimator(
		const nlohmann::json& estimator_json,
		std::optional<estimation::ExtendedKalmanEstimator>& estimator)
	{
		estimator = make_extended_kalman_estimator(fetch_estimator_type(estimator_json), estimator_json);
	}

	estimation::EstimationManager parse_estimation_manager(
		const nlohmann::json& config,
		bool trim_flag)
	{
		validate_estimator(config, trim_flag);
		estimation::EstimationManager estimation_manager;

		switch (fetch_estimator_type(config)) {
			case estimation::EstimatorType::None:
				break;

			case estimation::EstimatorType::LinearKalmanFilter:
				parse_linear_kalman_estimator(config, estimation_manager.linear_kalman_estimator);
				break;

			case estimation::EstimatorType::ExtendedKalmanFilter:
				parse_extended_kalman_estimator(config, estimation_manager.extended_kalman_estimator);
				break;

			default:
				throw std::runtime_error("json::parse_estimation_manager unknown estimator type");
		}

		return estimation_manager;
	}

} // namespace json
