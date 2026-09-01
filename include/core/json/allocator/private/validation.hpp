#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_allocator(const nlohmann::json& config);

}
