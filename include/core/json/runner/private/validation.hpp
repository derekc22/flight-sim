#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_simulation_hz_json(const nlohmann::json& hz_json, const nlohmann::json& enable_json);
    void validate_enable_json(const nlohmann::json& enable_json);
    void validate_simulation_json(const nlohmann::json& simulation_json);
    void validate_log_hz(double hz, const nlohmann::json& enable_json);
    void validate_logging_json(const nlohmann::json& logging_json);

}
