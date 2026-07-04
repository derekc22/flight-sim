#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/estimation/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/estimators/ekf/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/estimators/lkf/public.hpp"

namespace json {

    template <typename Estimator, typename EstimatorClass, typename EstimatorParameters, typename EstimatorClassInput>
    EstimatorClass make_stateful_estimator(const EstimatorParameters& params);

    estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(const nlohmann::json& estimator_json);
    estimation::LinearKalmanEstimatorFunction make_linear_kalman_estimator(estimation::EstimatorType estimator_type, const nlohmann::json& estimator_json);
    estimation::ExtendedKalmanEstimatorFunction make_extended_kalman_estimator(estimation::EstimatorType estimator_type, const nlohmann::json& estimator_json);
    estimation::EstimatorType map_estimator_type(const std::string& estimator_type_str);
    estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_json);
    void parse_linear_kalman_estimator(const nlohmann::json& estimator_json, estimation::LinearKalmanEstimatorFunction& estimator, estimation::EstimatorType& estimator_type);
    void parse_extended_kalman_estimator(const nlohmann::json& estimator_json, estimation::ExtendedKalmanEstimatorFunction& estimator, estimation::EstimatorType& estimator_type);
    estimation::EstimationProperties parse_estimation_properties(const nlohmann::json& config);
}

#include "core/json/estimation/private.tpp"
