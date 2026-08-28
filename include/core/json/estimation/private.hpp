#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "simulation/estimation/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/estimators/ekf/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/estimators/lkf/public.hpp"

namespace json {

    template <typename Estimator, typename EstimatorClass, typename EstimatorParameters, typename EstimatorClassInput>
    EstimatorClass make_stateful_estimator(const EstimatorParameters& params);

    estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(const nlohmann::json& estimator_json);

    estimation::LinearKalmanEstimator make_linear_kalman_estimator(estimation::EstimatorType estimator_type, const nlohmann::json& estimator_json);
    estimation::ExtendedKalmanEstimator make_extended_kalman_estimator(estimation::EstimatorType estimator_type, const nlohmann::json& estimator_json);

    estimation::EstimatorType map_estimator_type(const std::string& estimator_type_str);
    estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_json);

    void parse_linear_kalman_estimator(const nlohmann::json& estimator_json, estimation::LinearKalmanEstimator& estimator, estimation::EstimatorType& estimator_type);
    void parse_extended_kalman_estimator(const nlohmann::json& estimator_json, estimation::ExtendedKalmanEstimator& estimator, estimation::EstimatorType& estimator_type);
    void validate_estimator(const nlohmann::json& estimator_json, bool trim_flag);

    estimation::EstimationManager parse_estimation_manager(const nlohmann::json& config, bool trim_flag);
}

#include "core/json/estimation/private.tpp"
