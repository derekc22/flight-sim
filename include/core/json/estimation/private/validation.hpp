#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_estimator(const nlohmann::json& estimator_json, bool trim_flag);

}
