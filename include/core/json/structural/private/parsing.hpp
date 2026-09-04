#pragma once
#include "simulation/structural/public/manager.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	structural::StructuralManager parse_structural_manager(const nlohmann::json& config);
}
