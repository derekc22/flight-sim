#pragma once
#include <nlohmann/json.hpp>
#include "simulation/allocator/public.hpp"

namespace json {

    void validate_allocator(const nlohmann::json& config);
    allocator::AllocatorProperties parse_allocator_properties(const nlohmann::json& config);
}
