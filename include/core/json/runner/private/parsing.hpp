#pragma once
#include <nlohmann/json.hpp>
#include "simulation/runner/public/data/types.hpp"

namespace json {

    runner::JSONOptions parse_runner_options(const nlohmann::json& config);

}
