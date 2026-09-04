#pragma once
#include "simulation/runner/public/data/types.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	runner::JSONOptions parse_runner_config();

}
