#pragma once
#include <nlohmann/json.hpp>
#include "simulation/runner/public.hpp"

namespace json {

    void validate_simulation_hz_json(const nlohmann::json& hz_json, const nlohmann::json& enable_json);
    void validate_enable_json(const nlohmann::json& enable_json);
    runner::JSONOptions parse_runner_options(const nlohmann::json& config);

}
