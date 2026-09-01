#pragma once
#include <nlohmann/json.hpp>
#include "simulation/allocator/public/manager.hpp"

namespace json {

    allocator::AllocatorManager parse_allocator_manager(const nlohmann::json& config);

}
