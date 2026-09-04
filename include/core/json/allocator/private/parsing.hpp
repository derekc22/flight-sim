#pragma once
#include "simulation/allocator/public/manager.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	allocator::AllocatorManager parse_allocator_manager(const nlohmann::json& config);

}
