#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/estimation/private/parsing.hpp"
#include "core/json/estimation/private/validation.hpp"

namespace json {

    void validate_estimator(const nlohmann::json& estimator_json, bool trim_flag) {
        if (fetch_estimator_type(estimator_json) == estimation::EstimatorType::LinearKalmanFilter && !trim_flag) {
            throw std::runtime_error("json::validate_estimator: LinearKalmanFilter requires trim");
        }
    }

}
