#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "core/json/estimation/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/estimation/kalman/estimators/ekf/public.hpp"
#include "simulation/estimation/kalman/estimators/lkf/public.hpp"

namespace json {

    template <typename EstimatorType, typename EstimatorClass, typename EstimatorParametersType>
    EstimatorClass make_stateful_kalman_filter_estimator(const EstimatorParametersType& params);

    estimation::LinearKalmanFilterParameters parse_linear_kalman_filter_parameters(const nlohmann::json& estimator_json);
    estimation::KalmanFilterEstimator make_kalman_filter_estimator(estimation::EstimatorType estimator_type, const nlohmann::json& estimator_json);
    estimation::EstimatorType map_estimator_type(const std::string& estimator_type_str);
    estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_json);
    void parse_kalman_filter_estimator(const nlohmann::json& estimator_json, estimation::KalmanFilterEstimator& estimator, estimation::EstimatorType& estimator_type);
    estimation::EstimationProperties parse_estimation_properties(const nlohmann::json& config);
}

#include "core/json/estimation/private.tpp"
