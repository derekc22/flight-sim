#pragma once
#include <nlohmann/json.hpp>
#include "simulation/structural/public/manager.hpp"

namespace json {

	structural::StructuralManager parse_structural_manager(const nlohmann::json& config);
}
