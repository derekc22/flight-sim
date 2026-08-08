#pragma once
#include <nlohmann/json.hpp>
#include "simulation/structural/public.hpp"

namespace json {

    structural::StructuralProperties parse_structural_properties(const nlohmann::json& config);
}
