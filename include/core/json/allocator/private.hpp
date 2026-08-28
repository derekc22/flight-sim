#pragma once
#include <nlohmann/json.hpp>
#include "simulation/allocator/public.hpp"

namespace json {

    void validate_allocator(const nlohmann::json& config);
    allocator::AllocatorManager parse_allocator_manager(const nlohmann::json& config);
}
