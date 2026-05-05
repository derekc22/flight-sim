#pragma once
#include <nlohmann/json.hpp>
#include "simulation/estimation/estimation.hpp"

namespace json {

    estimation::EstimationProperties parse_estimation_config();
    estimation::EstimatorType fetch_estimator_type(const nlohmann::json& estimator_config);
}
